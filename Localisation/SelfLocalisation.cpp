#include "SelfLocalisation.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Tools/Math/General.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "nubotdataconfig.h"
#include "nubotconfig.h"

#include <assert.h>


#define MULTIPLE_MODELS_ON 1
#define AMBIGUOUS_CORNERS_ON 1
#define SHARED_BALL_ON 0
#define TWO_OBJECT_UPDATE_ON 0

#define BOX_LENGTH_X 160
#define BOX_LENGTH_Y 160
#define X_BOUNDARY 300
#define Y_BOUNDARY 200

#define CENTER_CIRCLE_ON 1 

//#define debug_out cout
#if DEBUG_LOCALISATION_VERBOSITY > 0
#define debug_out debug_file
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

using namespace mathGeneral;

typedef std::vector<StationaryObject> StationaryObjects;
typedef StationaryObjects::iterator StationaryObjectsIt;
typedef StationaryObjects::const_iterator StationaryObjectsConstIt;

typedef std::vector<MobileObject> MobileObjects;
typedef MobileObjects::iterator MobileObjectsIt;
typedef MobileObjects::const_iterator MobileObjectsConstIt;

typedef std::vector<AmbiguousObject> AmbiguousObjects;
typedef AmbiguousObjects::iterator AmbiguousObjectsIt;
typedef AmbiguousObjects::const_iterator AmbiguousObjectsConstIt;

// Constant value initialisation
const float SelfLocalisation::c_LargeAngleSD = PI/2;   //For variance check
const float SelfLocalisation::c_OBJECT_ERROR_THRESHOLD = 0.3f;
const float SelfLocalisation::c_OBJECT_ERROR_DECAY = 0.94f;

const float SelfLocalisation::c_RESET_SUM_THRESHOLD = 3.0f; // 3 // then 8.0 (home)
const int SelfLocalisation::c_RESET_NUM_THRESHOLD = 2;

// Object distance measurement error weightings (Constant)
const float SelfLocalisation::c_obj_theta_variance = 0.1f*0.1f;        // (0.01 rad)^2
const float SelfLocalisation::c_obj_range_offset_variance = 10.0f*10.0f;     // (10cm)^2
const float SelfLocalisation::c_obj_range_relative_variance = 0.20f*0.20f;   // 20% of range added

const float SelfLocalisation::c_centre_circle_heading_variance = (float)(deg2rad(20)*deg2rad(20)); // (10 degrees)^2

const float SelfLocalisation::sdTwoObjectAngle = 0.05f; //Small! error in angle difference is normally very small

/*! @brief Constructor
    @param playerNumber The player number of the current robot/system. This assists in choosing reset positions.
 */

SelfLocalisation::SelfLocalisation(int playerNumber, const LocalisationSettings& settings): m_timestamp(0), m_settings(settings)
{
    m_hasGps = false;
    m_previously_incapacitated = true;
    m_previous_game_state = GameInformation::InitialState;
    m_currentFrameNumber = 0;

    m_amILost = true;
    m_lostCount = 100;
    m_timeSinceFieldObjectSeen = 0;

    m_models.clear();
    //m_models.reserve(c_MAX_MODELS);

    m_pastAmbiguous.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);

    initSingleModel(67.5f, 0, mathGeneral::PI);

    #if DEBUG_LOCALISATION_VERBOSITY > 0
        std::stringstream debugLogName;
        debugLogName << DATA_DIR;
        if(playerNumber) debugLogName << playerNumber;
        debugLogName << "Localisation.log";
        debug_file.open(debugLogName.str().c_str());
        debug_file.clear();
        debug_file << "Localisation" << std::endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
    return;
}

SelfLocalisation::SelfLocalisation(int playerNumber): m_timestamp(0)
{
    m_hasGps = false;
    m_previously_incapacitated = true;
    m_previous_game_state = GameInformation::InitialState;
    m_currentFrameNumber = 0;
	
    m_amILost = true;
    m_lostCount = 100;
    m_timeSinceFieldObjectSeen = 0;

    m_models.clear();

    // Set default settings
    m_settings.setBranchMethod(LocalisationSettings::branch_exhaustive);
    m_settings.setPruneMethod(LocalisationSettings::prune_merge);

    m_pastAmbiguous.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);

    initSingleModel(67.5f, 0, mathGeneral::PI);

    #if DEBUG_LOCALISATION_VERBOSITY > 0
        std::stringstream debugLogName;
        debugLogName << DATA_DIR;
        if(playerNumber) debugLogName << playerNumber;
        debugLogName << "Localisation.log";
        debug_file.open(debugLogName.str().c_str());
        debug_file.clear();
        debug_file << "Localisation" << std::endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
    return;
}

/*! @brief Copy Constructor
    @param source The source localisation system from which to copy
 */
SelfLocalisation::SelfLocalisation(const SelfLocalisation& source): TimestampedData(), m_settings(source.m_settings)
{
    *this = source;
}

/*! @brief Copy operator
    @param source The source localisation system from which to copy

    @return Returns the current localisation following the copy.
 */
SelfLocalisation& SelfLocalisation::operator=(const SelfLocalisation& source)
{
    if (this != &source) // protect against invalid self-assignment
    {
        m_timestamp = source.m_timestamp;
        m_currentFrameNumber = source.m_currentFrameNumber;
        m_previously_incapacitated = source.m_previously_incapacitated;
        m_previous_game_state = source.m_previous_game_state;
        m_frame_log.str(source.m_frame_log.str());
        m_gps = source.m_gps;
        m_compass = source.m_compass;
        m_hasGps = source.m_hasGps;
        m_settings = source.m_settings;

        clearModels();
        m_models.clear();
        for (ModelContainer::const_iterator model_it = source.m_models.begin(); model_it != source.m_models.end(); ++model_it)
        {
            m_models.push_back(new Model(*(*model_it)));
        }
    }
    // by convention, always return *this
    return *this;
}

/*! @brief Destructor
 */
SelfLocalisation::~SelfLocalisation()
{
    clearModels();
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_file.close();
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0
}


//--------------------------------- MAIN FUNCTIONS  ---------------------------------//

/*! @brief Process function
    Processes the data from the current frame to determine the new estimation of the robots position.

    @param sensor_data Data from the robots sensors. These are used to determine the odometry of the robot.
    @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
    @param gameInfo Contains information about the game. This is used to determine resetting triggers when set conditions are met.
    @param teamInfo The team information. This contains the information received from other robots.
 */
void SelfLocalisation::process(NUSensorsData* sensor_data, FieldObjects* fobs, const GameInformation* gameInfo, const TeamInformation* teamInfo)
{

    m_frame_log.str(""); // Clear buffer.
    if (sensor_data == NULL or fobs == NULL)
        return;

    // Calculate time passed since previous frame.
    float time_increment = sensor_data->CurrentTime - m_timestamp;
    m_timestamp = sensor_data->CurrentTime;
    m_currentFrameNumber++;

#if LOC_SUMMARY > 0
    m_frame_log << "Frame " << m_currentFrameNumber << " Time: " << m_timestamp << std::endl;
#endif

    // Check if processing is required.
    bool processing_required = CheckGameState(sensor_data->isIncapacitated(), gameInfo);

    // retrieve gps data
    if (sensor_data->getGps(m_gps) and sensor_data->getCompass(m_compass))
    {
        m_hasGps = true;
    }

    // Retrieve odometry data
    vector<float> odo;
    bool odom_ok = sensor_data->getOdometry(odo);

    if(processing_required == false)
    {
        #if LOC_SUMMARY > 0
        m_frame_log << "Processing Cancelled." << std::endl;
        #endif
        return;
    }

#ifndef USE_VISION
    // If vision is disabled, gps coordinates are used in its place to trach location.
    vector<float> gps;
    float compass;
    if (sensor_data->getGps(gps) and sensor_data->getCompass(compass))
    {
        #if LOC_SUMMARY > 0
        m_frame_log << "Setting position from GPS: (" << gps[0] << "," << gps[1] << "," << compass << ")" << std::endl;
        #endif
        fobs->self.updateLocationOfSelf(gps[0], gps[1], compass, 0.1, 0.1, 0.01, false);
        return;
    }
#else

    if (odom_ok)
    {
        float fwd = odo[0];
        float side = odo[1];
        float turn = odo[2];
        // perform odometry update and change the variance of the model

        #if LOC_SUMMARY > 0
        m_frame_log << "Time Update - Odometry: (" << fwd << "," << side << "," << turn << ")";
        m_frame_log << " Time Increment: " << time_increment << std::endl;
        #endif
        // Hack... Sometimes we get really big odometry turn values that are not real.
        // TODO: See if this can be removed.
        if(fabs(turn) > 1.0f) turn = 0;

        doTimeUpdate(fwd, side, turn, time_increment);

        #if LOC_SUMMARY > 0
            m_frame_log << std::endl << "Result: " << getBestModel()->summary(false);
        #endif
    }

    #if LOC_SUMMARY > 0
    m_frame_log << "Observation Update:" << std::endl;
    int objseen = 0;
    bool seen;
    std::vector<StationaryObject>::iterator s_it = fobs->stationaryFieldObjects.begin();
    while(s_it != fobs->stationaryFieldObjects.end())
    {
        seen = s_it->isObjectVisible();
        if(seen)
            ++objseen;
        ++s_it;
    }
    m_frame_log << "Stationary Objects: " << objseen << std::endl;

    objseen = 0;
    for (int i=0; i < fobs->mobileFieldObjects.size(); i++)
    {
        if(fobs->mobileFieldObjects[i].isObjectVisible()) ++objseen;
    }
    m_frame_log << "Mobile Objects: " << objseen << std::endl;
    m_frame_log << "Ambiguous Objects: " << fobs->ambiguousFieldObjects.size() << std::endl;
    #endif

    ProcessObjects(fobs, time_increment);

    // Check for model reset. -> with multiple models just remove if not last one??
    // Need to re-do reset to be model specific.
    int num_models = getNumActiveModels();
    int num_reset = CheckForOutlierResets();

//    if(num_reset == num_models)
//        ProcessObjects(fobs, time_increment);

    // clip models back on to field.
    clipActiveModelsToField();

    // Store WM Data in Field Objects.
    //int bestModelID = getBestModelID();
    // Get the best model to use.
    WriteModelToObjects(getBestModel(), fobs);


    #if LOC_SUMMARY > 0
    m_frame_log << "num Models: " << num_models << " num reset: " << num_reset << std::endl;
    m_frame_log << std::endl <<  "Final Result: " << ModelStatusSummary();
    #endif
#endif
}

/*! @brief Process objects
    Processes the field objects and perfroms the correction updates required from the observations.

    @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
    @param time_increment The time that has elapsed since the previous localisation frame.

 */
void SelfLocalisation::ProcessObjects(FieldObjects* fobs, float time_increment)
{
    int numUpdates = 0;
    int updateResult;
    int usefulObjectCount = 0;

#if DEBUG_LOCALISATION_VERBOSITY > 2
    if(numUpdates == 0 )
    {
        debug_out  <<"[" << m_timestamp << "]: Update Starting." << endl;
        for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
        {
            if((*model_it)->active() == false) continue;
            debug_out  << "[" << m_timestamp << "]: Model[" << (*model_it)->id() << "]";
            debug_out  << " [alpha = " << (*model_it)->alpha() << "]";
            debug_out  << " Robot X: " << (*model_it)->mean(SelfModel::states_x);
            debug_out  << " Robot Y: " << (*model_it)->mean(SelfModel::states_y);
            debug_out  << " Robot Theta: " << (*model_it)->mean(SelfModel::states_heading) << endl;
        }
    }
#endif // DEBUG_LOCALISATION_VERBOSITY > 2

    // Proccess the Stationary Known Field Objects
    StationaryObjectsIt currStat(fobs->stationaryFieldObjects.begin());
    StationaryObjectsConstIt endStat(fobs->stationaryFieldObjects.end());

    // all objects at once.
    std::vector<StationaryObject*> update_objects;
    unsigned int objectsAdded = 0;
    for(; currStat != endStat; ++currStat)
    {
        if(currStat->isObjectVisible() == false) continue; // Skip objects that were not seen.
#if CENTER_CIRCLE_ON
		update_objects.push_back(&(*currStat));
		objectsAdded++;
#else
		if(!((*currStat).getName() == fobs->stationaryFieldObjects[FieldObjects::FO_CORNER_CENTRE_CIRCLE].getName()))
		{
			update_objects.push_back(&(*currStat));
			objectsAdded++;
		}
#endif
    }
    updateResult = multipleLandmarkUpdate(update_objects);
    numUpdates+=objectsAdded;
    usefulObjectCount+=objectsAdded;

    // Two Object update
#if TWO_OBJECT_UPDATE_ON
    if( fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible()
        && fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible())
        {
            doTwoObjectUpdate(fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST],
                              fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]);
        }
    if( fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible()
        && fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible())
        {
            doTwoObjectUpdate(fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST],
                              fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]);
        }
#endif

        NormaliseAlphas();

#if MULTIPLE_MODELS_ON
        bool blueGoalSeen = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() || fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible();
        bool yellowGoalSeen = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() || fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible();
        removeAmbiguousGoalPairs(fobs->ambiguousFieldObjects, yellowGoalSeen, blueGoalSeen);
        // Do Ambiguous objects.
        AmbiguousObjectsIt currAmb(fobs->ambiguousFieldObjects.begin());
        AmbiguousObjectsConstIt endAmb(fobs->ambiguousFieldObjects.end());
        for(; currAmb != endAmb; ++currAmb){
            if(currAmb->isObjectVisible() == false) continue; // Skip objects that were not seen.
            std::vector<int> possible_ids = currAmb->getPossibleObjectIDs();
            std::vector<StationaryObject*> poss_obj;
            poss_obj.reserve(possible_ids.size());
            for(std::vector<int>::iterator pos_it = possible_ids.begin(); pos_it != possible_ids.end(); ++pos_it)
            {
                poss_obj.push_back(&(fobs->stationaryFieldObjects[(*pos_it)]));
            }

            updateResult = ambiguousLandmarkUpdate((*currAmb), poss_obj);
            NormaliseAlphas();
            numUpdates++;
            if(currAmb->getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN or currAmb->getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
                usefulObjectCount++;
        }
#endif // MULTIPLE_MODELS_ON
        PruneModels();

#if DEBUG_LOCALISATION_VERBOSITY > 1
        for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
        {
            if( (*model_it)->active() )
            {
                debug_out   << "Model : " << (*model_it)->id() << " Pos  : " << (*model_it)->mean(Model::states_x) << ", "
                        << (*model_it)->mean(Model::states_y) << "," << (*model_it)->mean(Model::states_heading) << std::endl;
            }
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    
        if (usefulObjectCount > 0)
            m_timeSinceFieldObjectSeen = 0;
        else
            m_timeSinceFieldObjectSeen += time_increment;

#if DEBUG_LOCALISATION_VERBOSITY > 2
        const SelfModel* bestModel = getBestModel();
        if(numUpdates > 0)
        {
            for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
            {
                if( (*model_it)->active() == false) continue;
                debug_out  << "[" << m_timestamp << "]: Model[" << (*model_it)->id() << "]";
                debug_out  << " [alpha = " << (*model_it)->alpha() << "]";
                debug_out  << " Robot X: " << (*model_it)->mean(Model::states_x);
                debug_out  << " Robot Y: " << (*model_it)->mean(Model::states_y);
                debug_out  << " Robot Theta: " << (*model_it)->mean(Model::states_heading) << endl;
            }
            debug_out  << "[" << m_timestamp << "]: Best Model";
            debug_out  << " [alpha = " << bestModel->alpha() << "]";
            debug_out  << " Robot X: " << bestModel->mean(Model::states_x);
            debug_out  << " Robot Y: " << bestModel->mean(Model::states_y);
            debug_out  << " Robot Theta: " << bestModel->mean(Model::states_heading) << endl;
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 2	
}

/*! @brief Remove ambiguous pairs

    Scans the ambiguous objects and determines if too many of a similar object has been seen. If there are too many of the same object seen the
    objects are removed and not processed.

    @param ambiguousobjects A vector contianing all o fthe ambiguous objects.
    @param yellow_seen Boolean value. True if a fixed yellow goal has been seen. False if it has not.
    @param blue_seen Boolean value. True if a fixed blue goal has been seen. False if it has not.

 */
void SelfLocalisation::removeAmbiguousGoalPairs(std::vector<AmbiguousObject>& ambiguousobjects, bool yellow_seen, bool blue_seen)
{
    // Do Ambiguous objects.
    AmbiguousObjectsIt currAmb(ambiguousobjects.begin());
    AmbiguousObjectsConstIt endAmb(ambiguousobjects.end());
    AmbiguousObjectsIt blueGoal = ambiguousobjects.end();
    AmbiguousObjectsIt yellowGoal = ambiguousobjects.end();
    bool toomanyblue = blue_seen;
    bool toomanyyellow = yellow_seen;

    for(; currAmb != endAmb; ++currAmb)
    {
        if(currAmb->isObjectVisible() == false) continue; // Skip objects that were not seen.
        if(currAmb->getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
        {
            if(yellowGoal == endAmb)
            {
                yellowGoal = currAmb;
            }
            else
            {
                toomanyyellow = true;
                currAmb->setIsVisible(false);
            }
        }
        if(toomanyyellow) yellowGoal->setIsVisible(false);

        if(currAmb->getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
        {
            if(blueGoal == endAmb)
            {
                blueGoal = currAmb;
            }
            else
            {
                toomanyblue=true;
                currAmb->setIsVisible(false);
            }
        }
        if(toomanyblue) blueGoal->setIsVisible(false);
    }
    return;
}

/*! @brief Writes current model to the objects

    Writes the current model to the field objects.

    @param model The model to be written.
    @param fieldObjects The objects to which the state will be written.

 */
void SelfLocalisation::WriteModelToObjects(const SelfModel* model, FieldObjects* fieldObjects)
{

    // Check if lost.
    bool lost = false;
    if (m_lostCount > 20)
        lost = true;
    fieldObjects->self = model->GenerateSelfState();
}


/*! @brief Checks the current state of the game and determines actions to be taken.

    Checks the current state of the game and determines if processing will be done. Sencondly also triggers a number of resets
    based on the game condiitions.

    @param currently_incapacitated Indicator for the robots condition. True if it has fallen or has otherwise been disrupted. False if it has not.
    @param game_info Information about the current state of the game.

    @return Returns True if processing should be performed. False if it should not.

 */
bool SelfLocalisation::CheckGameState(bool currently_incapacitated, const GameInformation* game_info)
{
    GameInformation::TeamColour team_colour = game_info->getTeamColour();
    GameInformation::RobotState current_state = game_info->getCurrentState();
    /*
    if (currently_incapacitated)
    {   // if the robot is incapacitated there is no point running localisation
        m_previous_game_state = current_state;
        m_previously_incapacitated = true;
        #if LOC_SUMMARY > 0
        m_frame_log << "Robot is incapscitated." << std::endl;
        #endif
        return false;
    }
    */

    if (current_state == GameInformation::InitialState or current_state == GameInformation::FinishedState or current_state == GameInformation::PenalisedState)
    {   // if we are in initial, finished, penalised or substitute states do not do localisation
        m_previous_game_state = current_state;
        m_previously_incapacitated = currently_incapacitated;
        #if LOC_SUMMARY > 0
        m_frame_log << "Robot in non-processing state: " << GameInformation::stateName(current_state) << std::endl;
        #endif
        return false;
    }
    
    if (current_state == GameInformation::ReadyState)
    {   // if are in ready. If previously in initial or penalised do a reset. Also reset if fallen over.
        if (m_previous_game_state == GameInformation::InitialState)
            doInitialReset(team_colour);
        else if (m_previous_game_state == GameInformation::PenalisedState)
            doPenaltyReset();
        else if (m_previously_incapacitated and not currently_incapacitated)
            doFallenReset();
    }
    else if (current_state == GameInformation::SetState)
    {   // if we are in set look for manual placement, if detected then do a reset.
        if (m_previously_incapacitated and not currently_incapacitated)
            doSetReset(team_colour, game_info->getPlayerNumber(), game_info->haveKickoff());
    }
    else
    {   // if we are playing. If previously penalised do a reset. Also reset if fallen over
        if (m_previous_game_state == GameInformation::PenalisedState)
            doPenaltyReset();
        /*
        else if (m_previously_incapacitated and not currently_incapacitated)
            doFallenReset();
            */
    }
    
    m_previously_incapacitated = currently_incapacitated;
    m_previous_game_state = current_state;
    return true;
}

void SelfLocalisation::initSingleModel(float x, float y, float heading)
{
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Initialising single model." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    clearModels();
    std::vector<Moment> positions;
    positions.reserve(1);
    Moment temp(Model::states_total);
    temp.setMean(mean_matrix(x,y,heading));
    temp.setCovariance(covariance_matrix(150.0f, 100.0f, 2*PI));
    positions.push_back(temp);
    InitialiseModels(positions);
}

void SelfLocalisation::doInitialReset(GameInformation::TeamColour team_colour)
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset leaving initial." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing initial->ready reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    
    clearModels();
    
    // The models are always in the team's own half
    // Model 0: On left sideline facing in, 1/3 from half way
    // Model 1: On left sideline facing in, 2/3 from half way
    // Model 2: On right sideline facing in, 1/3 from half way
    // Model 3: On right sideline facing in, 2/3 from half way
    // Model 4: In centre of own half facing opponents goal
	 
    float left_y = 200.0;
    float left_heading = -PI/2;
    float right_y = -200.0;
    float right_heading = PI/2;
    
    float front_x = -300.0/4.0f;
    float centre_x = -300.0/2.0f;
    float centre_heading = 0;
    float back_x = -300*(3.0f/4.0f);
    if (team_colour == GameInformation::RedTeam)
    {   // flip the invert the x for the red team and set the heading to PI
        front_x = -front_x;
        centre_x = -centre_x;
        centre_heading = PI;
        back_x = -back_x;
    }
    
    Matrix cov_matrix = covariance_matrix(50.0f, 15.0f, 0.2f);
    Moment temp(Model::states_total);
    temp.setCovariance(cov_matrix);
    std::vector<Moment> positions;
    positions.reserve(5);

    // Position 1
    temp.setMean(mean_matrix(front_x, right_y, right_heading));
    positions.push_back(temp);

    // Position 2
    temp.setMean(mean_matrix(back_x, right_y, right_heading));
    positions.push_back(temp);

    // Position 3
    temp.setMean(mean_matrix(front_x, left_y, left_heading));
    positions.push_back(temp);

    // Position 4
    temp.setMean(mean_matrix(back_x, left_y, left_heading));
    positions.push_back(temp);

    // Postition 5
    temp.setCovariance(covariance_matrix(100.0f, 150.0f, PI/2.0f));
    temp.setMean(mean_matrix(centre_x, 0.0f, centre_heading));
    positions.push_back(temp);

    InitialiseModels(positions);

    return;
}

void SelfLocalisation::doSetReset(GameInformation::TeamColour team_colour, int player_number, bool have_kickoff)
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset due to manual positioning." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing manual position reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    clearModels();
    float x, y, heading;
    const float position_sd = 15;
    const float heading_sd = 0.1;
    int num_models;
    std::vector<Moment> positions;
    Moment temp(Model::states_total);
    temp.setCovariance(covariance_matrix(position_sd, position_sd, heading_sd));
    if (player_number == 1)
    {   // if we are the goal keeper and we get manually positioned we know exactly where we will be put
        num_models = 1;
        x = -300;
        y = 0; 
        heading = 0;
        if (team_colour == GameInformation::RedTeam)
            swapFieldStateTeam(x, y, heading);
        temp.setMean(mean_matrix(x,y,heading));
        positions.push_back(temp);
    }
    else
    {   // if we are a field player and we get manually positioned we could be in a three different places
        if (have_kickoff)
        {   // the attacking positions are on the circle or on either side of the penalty spot
            num_models = 3;
            // on the circle
            x = -60;
            y = 0;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            // on the left of the penalty spot
            x = -120;
            y = 70;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            // on the right of the penalty spot
            x = -120;
            y = -70;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            
        }
        else
        {   // the defensive positions are on either corner of the penalty box
            num_models = 2;
            // top penalty box corner
            x = -240;
            y = 110;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
            // bottom penalty box corner
            x = -240;
            y = -110;
            heading = 0;
            if (team_colour == GameInformation::RedTeam)
                swapFieldStateTeam(x, y, heading);
            temp.setMean(mean_matrix(x,y,heading));
            positions.push_back(temp);
        }
    }
    
    // Add the models.
    InitialiseModels(positions);
    return;
}

void SelfLocalisation::doPenaltyReset()
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset due to penalty." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing penalty reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0

    clearModels();

    std::vector<Moment> positions;
    positions.reserve(2);
    Moment temp(Model::states_total);
    temp.setCovariance(covariance_matrix(75.0f, 25.0f, 0.35f));

    // setup model 0 as top 'T'
    temp.setMean(mean_matrix(0.0f, 200.0, -PI/2.0f));
    positions.push_back(temp);
    
    // setup model 1 as bottom 'T'
    temp.setMean(mean_matrix(0.0f, -200.0f, PI/2.0f));
    positions.push_back(temp);

    InitialiseModels(positions);
    return;
}

void SelfLocalisation::doFallenReset()
{
    #if LOC_SUMMARY > 0
    m_frame_log << "Reset due to fall." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing fallen reset." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {   // Increase heading uncertainty if fallen
        Matrix temp = (*model_it)->covariance();
        temp[0][0] += 5;        // Robot x
        temp[1][1] += 5;        // Robot y
        temp[2][2] += 0.707;     // Robot heading
        (*model_it)->setCovariance(temp);
    }
}

void SelfLocalisation::doReset()
{
    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out  << "Performing player reset." << endl;
    #endif // DEBUG_LOCALISATION_VERBOSITY > 0

    clearModels();

    std::vector<Moment> newPositions;
    newPositions.reserve(4);

    Moment temp(Model::states_total);

    temp.setCovariance(covariance_matrix(150.0f, 100.0f, 2*PI));

    // First position ->
    temp.setMean(mean_matrix(300.0f, 0.0f, PI));
    newPositions.push_back(temp);

    // Second Position ->
    temp.setMean(mean_matrix(-300.0f, 0.0f, 0.0f));
    newPositions.push_back(temp);

    // Third Position ->
    temp.setMean(mean_matrix(0.0f, 200.0f, -PI/2.0f));
    newPositions.push_back(temp);

    // Fourth Positions ->
    temp.setMean(mean_matrix(0.0f, -200.0f, PI/2.0f));
    newPositions.push_back(temp);

    InitialiseModels(newPositions);
    return;
}

void SelfLocalisation::doBallOutReset()
{
//#if DEBUG_LOCALISATION_VERBOSITY > 0
//    debug_out  << "Performing ball out reset." << endl;
//#endif // DEBUG_LOCALISATION_VERBOSITY > 0
//    // Increase uncertainty of ball position if it has gone out.. Cause it has probably been moved.
//    for (int modelNumber = 0; modelNumber < c_MAX_MODELS; modelNumber++){
//        if(m_models[modelNumber].active() == false) continue;
//        m_models[modelNumber].stateStandardDeviations[3][3] += 100.0; // 100 cm
//        m_models[modelNumber].stateStandardDeviations[4][4] += 60.0; // 60 cm
//        m_models[modelNumber].stateStandardDeviations[5][5] += 10.0;   // 10 cm/s
//        m_models[modelNumber].stateStandardDeviations[6][6] += 10.0;   // 10 cm/s
//    }
    return;
}

/*! @brief Changes the field state so that it will be the position for the other team */
void SelfLocalisation::swapFieldStateTeam(float& x, float& y, float& heading)
{
    x = -x;
    y = -y;
    heading = normaliseAngle(heading + PI);
}

/*! @brief Clips the model to a position on the field.
    @param theModel THe model to be clipped

    @return True if clipping was required. False if not.
*/
bool SelfLocalisation::clipModelToField(SelfModel* theModel)
{
    const double fieldXLength = 680.0;
    const double fieldYLength = 440.0;
    const double fieldXMax = fieldXLength / 2.0;
    const double fieldXMin = - fieldXLength / 2.0;
    const double fieldYMax = fieldYLength / 2.0;
    const double fieldYMin = - fieldYLength / 2.0;

    bool wasClipped = false;
    bool clipped;
    double prevX, prevY, prevTheta;
    prevX = theModel->mean(Model::states_x);
    prevY = theModel->mean(Model::states_y);
    prevTheta = theModel->mean(Model::states_heading);

    clipped = theModel->clipState(0, fieldXMin, fieldXMax);		// Clipping for robot's X
    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << theModel->id() << "]";
        debug_out  << " [alpha = " << theModel->alpha() << "]";
        debug_out  << " State(0) clipped.";
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << theModel->mean(Model::states_x);
        debug_out  << "," << theModel->mean(Model::states_y) << "," << theModel->mean(Model::states_heading) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1
    wasClipped = wasClipped || clipped;

    prevX = theModel->mean(Model::states_x);
    prevY = theModel->mean(Model::states_y);
    prevTheta = theModel->mean(Model::states_heading);

    clipped = theModel->clipState(1, fieldYMin, fieldYMax);		// Clipping for robot's Y

    #if DEBUG_LOCALISATION_VERBOSITY > 1
    if(clipped){
        debug_out  << "[" << m_timestamp << "]: Model[" << theModel->id() << "]";
        debug_out  << " [alpha = " << theModel->alpha() << "]";
        debug_out  << " State(1) clipped." << endl;
        debug_out  << " (" << prevX << "," << prevY << "," << prevTheta << ") -> (" << theModel->mean(Model::states_x);
        debug_out  << "," << theModel->mean(Model::states_y) << "," << theModel->mean(Model::states_heading) << ")" << endl;
    }
    #endif // DEBUG_LOCALISATION_VERBOSITY > 1

    wasClipped = wasClipped || clipped;

    return wasClipped;
}

/*! @brief Clips all of the models to the field.

    @return True if clipping was required. False if not required.
*/
bool SelfLocalisation::clipActiveModelsToField()
{
    bool wasClipped = false;
    bool modelClipped = false;
    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->active() == true){
            modelClipped = clipModelToField((*model_it));
            wasClipped = wasClipped || modelClipped;
        }
    }
    return wasClipped;
}

bool SelfLocalisation::doTimeUpdate(float odomForward, float odomLeft, float odomTurn, double timeIncrement)
{
    std::vector<float> odom(3,0.0f);
    odom[0] = odomForward;
    odom[1] = odomLeft;
    odom[2] = odomTurn;

    bool result = false;
    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->inactive()) continue; // Skip Inactive models.
        result = true;
        OdometryMotionModel odom_Model(0.07,0.00005,0.00005,0.000005);
        (*model_it)->TimeUpdate(odom, odom_Model, timeIncrement);
    }
    
    const SelfModel* bestModel = getBestModel();
    double rmsDistance = 0;
    double entropy = 0;
    double bestModelEntropy = 0;
	
    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->inactive()) continue; // Skip Inactive models.
        
        rmsDistance = pow (
                            pow((bestModel->mean(Model::states_x) - bestModel->mean(Model::states_x)),2.0f) +
                            pow((bestModel->mean(Model::states_y) - bestModel->mean(Model::states_y)),2.0f) +
                            pow((bestModel->mean(Model::states_heading) - bestModel->mean(Model::states_heading)),2.0f) , 0.5f );
        entropy += (rmsDistance * bestModel->alpha());
    }
	
    Matrix bestModelCovariance = bestModel->covariance();
	
    bestModelCovariance = bestModelCovariance;
    bestModelEntropy =  0.5 * ( 3 + 3*log(2 * PI ) + log(  determinant(bestModelCovariance) ) ) ;
	
    if(entropy > 100 && bestModel->alpha() < 50 )
        m_amILost = true;
    else if (entropy <= 100 && bestModelEntropy > 20)
        m_amILost = true;
    else
        m_amILost = false;
    
    if(m_amILost)
        m_lostCount++;
    else
        m_lostCount = 0;

    return result;
}

int SelfLocalisation::multipleLandmarkUpdate(std::vector<StationaryObject*>& landmarks)
{
    const unsigned int num_objects = landmarks.size();
    if(num_objects == 0) return 0;
    unsigned int numSuccessfulUpdates = 0;
    std::vector<StationaryObject*>::iterator currStat(landmarks.begin());
    std::vector<StationaryObject*>::const_iterator endStat(landmarks.end());
    Matrix locations(2*num_objects, 1, false);
    Matrix measurements(2*num_objects, 1, false);
    Matrix R_measurement(2*num_objects, 2*num_objects, false);
    int kf_return;
    std::vector<unsigned int> objIds;


    unsigned int measurementNumber = 0;
    for(; currStat != endStat; ++currStat)
    {
        const int index = 2*measurementNumber;

        // Locations
        locations[index][0] = (*currStat)->X();
        locations[index+1][0] = (*currStat)->Y();

        double flatObjectDistance = (*currStat)->measuredDistance() * cos((*currStat)->measuredElevation());
        measurements[index][0] = flatObjectDistance;
        measurements[index+1][0] = (*currStat)->measuredBearing();

        // R
        R_measurement[index][index] = c_obj_range_offset_variance + c_obj_range_relative_variance * pow(measurements[index][0], 2);
        R_measurement[index+1][index+1] = c_obj_theta_variance;

        objIds.push_back((*currStat)->getID());
        measurementNumber++;
    }

#if LOC_SUMMARY > 0
    m_frame_log << "Performing multiple object update." << std::endl;
    m_frame_log << "locations:" << std::endl;
    m_frame_log << locations << std::endl;
    m_frame_log << "measurements:" << std::endl;
    m_frame_log << measurements << std::endl;
    m_frame_log << "R_measurement:" << std::endl;
    m_frame_log << R_measurement << std::endl;

#endif

    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->active() == false) continue; // Skip Inactive models.
        kf_return = SelfModel::RESULT_OK;
        kf_return = (*model_it)->MultipleObjectUpdate(locations, measurements, R_measurement);

#if LOC_SUMMARY > 0
        Matrix temp = Matrix(measurements.getm(), 1, false);
        for(unsigned int j=0; j < measurements.getm(); j+=2)
        {
            const float dX = locations[j][0]-(*model_it)->mean(Model::states_x);
            const float dY = locations[j+1][0]-(*model_it)->mean(Model::states_y);
            temp[j][0] = sqrt(dX*dX + dY*dY);
            temp[j+1][0] = normaliseAngle(atan2(dY,dX) - (*model_it)->mean(Model::states_heading));
        }
        m_frame_log << std::endl << "Update model " << (*model_it)->id() << std::endl;
        m_frame_log << "Expected Measurements: " << std::endl << temp << std::endl;
        m_frame_log <<" Result: " << ((kf_return==Model::RESULT_OK)?"Successful":"Outlier") << std::endl;
#endif
        if(kf_return == SelfModel::RESULT_OUTLIER)
        {
            currStat = landmarks.begin();
            for(; currStat != endStat; ++currStat)
            {
                MeasurementError temp_error = calculateError(*(*currStat));

//                kf_return = m_models[modelID].fieldObjectmeas(flatObjectDistance, (*currStat)->measuredBearing(),(*currStat)->X(), (*currStat)->Y(),
//                                R_obj_range_offset, R_obj_range_relative, R_obj_theta);

                #if LOC_SUMMARY > 0
                m_frame_log << "Individual Update: " <<  (*currStat)->getName() << " Result: " << ((kf_return==Model::RESULT_OK)?"Successful":"Outlier") << std::endl;
                m_frame_log << "Following individual object updates: " << (*model_it)->summary(false);
                #endif
//                if(kf_return == Model::RESULT_OUTLIER)
//                {
//                    m_modelObjectErrors[modelID][(*currStat)->getID()] += 1.0;
//                }
            }
        }
    #if LOC_SUMMARY > 0

    #endif
        if(kf_return == SelfModel::RESULT_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}

int SelfLocalisation::landmarkUpdate(StationaryObject &landmark)
{
#if LOC_SUMMARY > 0
    m_frame_log << std::endl << "Known landmark update: " << landmark.getName() << std::endl;
#endif
    int kf_return;
    int numSuccessfulUpdates = 0;

    if(landmark.validMeasurement() == false)
    {
    #if LOC_SUMMARY > 0
        m_frame_log << "Skipping invalid." << std::endl;
    #endif
#if DEBUG_LOCALISATION_VERBOSITY > 0
        debug_out  <<"[" << m_timestamp << "] Skipping Bad Landmark Update: ";
        debug_out  << landmark.getName();
        debug_out  << " Distance = " << landmark.measuredDistance();
        debug_out  << " Bearing = " << landmark.measuredBearing();
        debug_out  << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 1
        return SelfModel::RESULT_OUTLIER;
    }

    double flatObjectDistance = landmark.measuredDistance() * cos(landmark.measuredElevation());

    MeasurementError temp_error;
    temp_error.setDistance(c_obj_range_offset_variance + c_obj_range_relative_variance * pow(flatObjectDistance,2));
    temp_error.setHeading(c_obj_theta_variance);

    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->active() == false) continue; // Skip Inactive models.

#if DEBUG_LOCALISATION_VERBOSITY > 2
        debug_out  <<"[" << m_timestamp << "]: Model[" << (*model_it)->id() << "] Landmark Update. ";
        debug_out  << "Object = " << landmark.getName();
        debug_out  << " Distance = " << landmark.measuredDistance();
        debug_out  << " Bearing = " << landmark.measuredBearing();
        debug_out  << " Location = (" << landmark.X() << "," << landmark.Y() << ")...";
#endif // DEBUG_LOCALISATION_VERBOSITY > 1

        if(landmark.measuredBearing() != landmark.measuredBearing())
	{
#if DEBUG_LOCALISATION_VERBOSITY > 0
            debug_out  << "ABORTED Object Update Bearing is NaN skipping object." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
            continue;
        }
        kf_return = SelfModel::RESULT_OK;


        kf_return = (*model_it)->MeasurementUpdate(landmark, temp_error);

//        kf_return = m_models[modelID].fieldObjectmeas(flatObjectDistance, landmark.measuredBearing(),landmark.X(), landmark.Y(),
//			distanceOffsetError, distanceRelativeError, bearingError);
//        if(kf_return == Model::RESULT_OUTLIER) m_modelObjectErrors[modelID][landmark.getID()] += 1.0;

#if DEBUG_LOCALISATION_VERBOSITY > 0
        if(kf_return != SelfModel::RESULT_OK)
        {
            Matrix estimated_measurement = (*model_it)->CalculateMeasurementPrediction(landmark.X(),landmark.Y());
            debug_out << "OUTLIER!" << endl;
            debug_out << "Model[" << (*model_it)->id() << "]: Outlier Detected - " << landmark.getName() << endl;
            debug_out << "Measured - Distance = " << landmark.measuredDistance() << " Bearing = " << landmark.measuredBearing() << endl;
            debug_out << "Expected - Distance = " << estimated_measurement[0][0] << " Bearing = " << estimated_measurement[1][0] << endl;
        }
        else
        {
            debug_out << "OK!" << endl;
        }
#endif // DEBUG_LOCALISATION_VERBOSITY > 1
    #if LOC_SUMMARY > 0
        m_frame_log << "Model " << (*model_it)->id() << " updated using " << landmark.getName() << " measurment." << std::endl;
        m_frame_log << "Measurement: Distance = " << flatObjectDistance << ", Heading = " << landmark.measuredBearing() <<std::endl;
        m_frame_log << "Position: X = " << landmark.X() << ", Y = " << landmark.Y() <<std::endl;
        m_frame_log << "Current State: " << (*model_it)->mean(Model::states_x) << ", " << (*model_it)->mean(Model::states_y) << ", " << (*model_it)->mean(Model::states_heading) << std::endl;

        if(m_hasGps)
        {
            float dX = landmark.X()-m_gps[0];
            float dY = landmark.Y()-m_gps[1];
            float Cc = cos(m_compass);
            float Ss = sin(m_compass);
            float x_gps = dX * Cc + dY * Ss;
            float y_gps =  -dX * Ss + dY * Cc;
            m_frame_log << "GPS Expected: (" << x_gps << "," << y_gps << ")" << std::endl;
        }

        float model_x = (*model_it)->mean(Model::states_x);
        float model_y = (*model_it)->mean(Model::states_y);
        float model_heading = (*model_it)->mean(Model::states_heading);

        float exp_dist = sqrt(pow(landmark.X() - model_x,2) + pow(landmark.Y() - model_y,2));
        float positionHeading = atan2(landmark.Y() - model_y, landmark.X() - model_x);
        float exp_heading = normaliseAngle(positionHeading - model_heading);
        m_frame_log << "Expected: " << exp_dist << ", " << exp_heading <<std::endl;
        m_frame_log << "Measured: " << flatObjectDistance << "," << landmark.measuredBearing() << std::endl;
        m_frame_log << "Result = ";
        m_frame_log << ((kf_return==Model::RESULT_OUTLIER)?"Outlier":"Success") << std::endl;
    #endif
        if(kf_return == SelfModel::RESULT_OK) numSuccessfulUpdates++;
    }
    return numSuccessfulUpdates;
}

int SelfLocalisation::doTwoObjectUpdate(StationaryObject &landmark1, StationaryObject &landmark2)
{
/*
#if LOC_SUMMARY > 0
    m_frame_log << std::endl << "Two object landmark update: " << landmark1.getName() << " & " << landmark2.getName() << std::endl;
#endif
    float totalAngle = landmark1.measuredBearing() - landmark2.measuredBearing();

    #if DEBUG_LOCALISATION_VERBOSITY > 0
    debug_out << "Two Object Update:" << endl;
    debug_out << landmark1.getName() << " - Bearing = " << landmark1.measuredBearing() << endl;
    debug_out << landmark2.getName() << " - Bearing = " << landmark2.measuredBearing() << endl;
    #endif
    for (int currID = 0; currID < c_MAX_MODELS; currID++){
        if(m_models[currID]->active())
        {
            m_models[currID].updateAngleBetween(totalAngle,landmark1.X(),landmark1.Y(),landmark2.X(),landmark2.Y(),sdTwoObjectAngle);
        }
    }
    */
    return 1;
}

/*! @brief Perfroms an ambiguous measurement update. This is a interface function to access a variety of methods.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::ambiguousLandmarkUpdate(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    if(m_settings.branchMethod() == LocalisationSettings::branch_exhaustive)
    {
        return ambiguousLandmarkUpdateExhaustive(ambiguousObject, possibleObjects);
    }
    else if (m_settings.branchMethod() == LocalisationSettings::branch_selective)
    {
        return ambiguousLandmarkUpdateSelective(ambiguousObject, possibleObjects);
    }
    else if (m_settings.branchMethod() == LocalisationSettings::branch_constraint)
    {
        return ambiguousLandmarkUpdateConstraint(ambiguousObject);
    }
    else if (m_settings.branchMethod() == LocalisationSettings::branch_probDataAssoc)
    {
        return ambiguousLandmarkUpdateProbDataAssoc(ambiguousObject, possibleObjects);
    }
    return 0;
}

/*! @brief Prunes the models using a selectable method. This is a interface function to access a variety of methods.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneModels()
{
    if(m_settings.pruneMethod() == LocalisationSettings::prune_merge)
    {
        MergeModels(c_MAX_MODELS_AFTER_MERGE);
    }
    else if (m_settings.pruneMethod() == LocalisationSettings::prune_max_likelyhood)
    {
        PruneMaxLikelyhood();
    }
    else if (m_settings.pruneMethod() == LocalisationSettings::prune_viterbi)
    {
        PruneViterbi(4);
    }
    else if (m_settings.pruneMethod() == LocalisationSettings::prune_nscan)
    {
        PruneNScan(4);
    }
    NormaliseAlphas();
    return 0;
}

/*! @brief Prunes the models using the maximum likelyhood method. This removes all but the mot probable model.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneMaxLikelyhood()
{
    return PruneViterbi(1);     // Max likelyhood is the equivalent of keeping the best 1 model.
}


struct model_ptr_cmp{
    bool operator()( SelfModel* lhs, SelfModel* rhs){
        return * lhs < * rhs;
    }
};

/*! @brief Prunes the models using the Viterbi method. This removes lower probability models to a maximum total models.
    @param order The number of models to be kept at the end for the process.
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneViterbi(unsigned int order)
{
    if(m_models.size() <= order) return 0;                      // No pruning required if less than maximum.
    m_models.sort(model_ptr_cmp());                                            // Sort, results in order smallest to largest.
    unsigned int num_to_remove = m_models.size() - order;       // Number of models that need to be removed.

    ModelContainer::iterator begin_remove = m_models.begin();   // Beginning of removal range
    ModelContainer::iterator end_remove = m_models.begin();
    std::advance(end_remove,num_to_remove);                     // End of removal range (not removed)


    std::for_each (begin_remove, end_remove, std::bind2nd(std::mem_fun(&Model::setActive), false));

    int num_removed = removeInactiveModels();    // Clear out all deactivated models.
    assert(m_models.size() == order);   // Result should have been achieved or something is broken.
    return num_removed;
}

bool AlphaSumPredicate( const ParentSum& a, const ParentSum& b )
{
        return a.second <  b.second;
}

/*! @brief Prunes the models using an N-Scan Pruning method.
    @param N The number of braches backward to prune the decision tree
    @return The number of models that were removed during this process.
*/
int SelfLocalisation::PruneNScan(unsigned int N)
{
    std::vector<ParentSum> results;
    // Sum the alphas of sibling branches from a common parent at branch K-N
    for (ModelContainer::iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        unsigned int parent_id = (*model_it)->history(N);
        float alpha = (*model_it)->alpha();
        bool added = false;
        if(parent_id == 0) continue;
        // Sort all of the models
        for(std::vector<ParentSum>::iterator resIt = results.begin(); resIt != results.end(); ++resIt)
        {
            // Assign to existing sum.
            if(resIt->first == parent_id)
            {
                resIt->second += alpha;
                added = true;
                break;
            }
        }
        // If not able to assign to an existing sum create a new one.
        if(!added)
        {
            ParentSum temp = make_pair(parent_id, alpha);
            results.push_back(temp);
        }
    }

    if(results.size() > 0)
    {
        std::sort(results.begin(), results.end(), AlphaSumPredicate); // Sort by alpha sum, smallest to largest
        unsigned int bestParentId = results.back().first;       // Get the parent Id of the best branch.

        // Remove all siblings not created from the best branch.
        for (ModelContainer::iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
        {
            if((*model_it)->history(N) != bestParentId)
            {
                (*model_it)->setActive(false);
            }
        }
    }
    removeInactiveModels();
    return 0;
}

/*! @brief Performs an ambiguous measurement update using the exhaustive process. This creates a new model for each possible location for the measurement.
    @return Returns RESULT_OK if measurment updates were sucessfully performed, RESULT_OUTLIER if they were not.
*/
int SelfLocalisation::ambiguousLandmarkUpdateExhaustive(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{

    const float outlier_factor = 0.0001;
    ModelContainer new_models;
    SelfModel* temp_mod;

    MeasurementError error = calculateError(ambiguousObject);

    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->inactive()) continue;
        unsigned int models_added = 0;
        for(std::vector<StationaryObject*>::const_iterator obj_it = possibleObjects.begin(); obj_it != possibleObjects.end(); ++obj_it)
        {
            temp_mod = new Model(*(*model_it), ambiguousObject, *(*obj_it), error, GetTimestamp());
            new_models.push_back(temp_mod);
#if LOC_SUMMARY > 0
            m_frame_log << "Model [" << (*model_it)->id() << " - > " << temp_mod->id() << "] Ambiguous object update: " << std::string((*obj_it)->getName());
            m_frame_log << "  Result: " << (temp_mod->active() ? "Valid update" : "Outlier");
            if(temp_mod->active())
            {
                m_frame_log << "Valid update (Alpha = " << temp_mod->alpha() << ")";
                models_added++;
            }
            else
            {
                m_frame_log << "Outlier";
            }
            m_frame_log << std::endl;
#endif
        }
        removeInactiveModels(new_models);
        if(models_added)
        {
            (*model_it)->setActive(false);
        }
        else
        {
            (*model_it)->setAlpha(outlier_factor * (*model_it)->alpha());
        }
    }
    if(new_models.size() > 0)
    {
        m_models.insert(m_models.end(), new_models.begin(), new_models.end());
        new_models.clear();
    }
    return SelfModel::RESULT_OUTLIER;
}

/*! @brief Performs an ambiguous measurement update using the constraint method.
    This method uses only options that should be visible.
    @return Returns RESULT_OK if measurment updates were sucessfully performed, RESULT_OUTLIER if they were not.
*/
int SelfLocalisation::ambiguousLandmarkUpdateConstraint(AmbiguousObject &ambiguousObject)
{

    const float outlier_factor = 0.0001;
    ModelContainer new_models;
    SelfModel* temp_mod, *curr_model;

    MeasurementError error = calculateError(ambiguousObject);

    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        curr_model = (*model_it);
        if(curr_model->inactive()) continue;
        vector<StationaryObject*> poss_objects;
        unsigned int models_added = 0;
        Self position = curr_model->GenerateSelfState();
        float headYaw;
        Blackboard->Sensors->getPosition(NUSensorsData::HeadYaw, headYaw);
        //! TODO: The FOV of the camera should NOT be hard-coded!
        poss_objects = Blackboard->Objects->filterToVisible(position, ambiguousObject, headYaw, 0.81f);

        for(std::vector<StationaryObject*>::const_iterator obj_it = poss_objects.begin(); obj_it != poss_objects.end(); ++obj_it)
        {
            temp_mod = new Model(*curr_model, ambiguousObject, *(*obj_it), error, GetTimestamp());
            new_models.push_back(temp_mod);

#if LOC_SUMMARY > 0
            m_frame_log << "Model [" << curr_model->id() << " - > " << temp_mod->id() << "] Ambiguous object update: " << std::string((*obj_it)->getName());
            m_frame_log << "  Result: " << (temp_mod->active() ? "Valid update" : "Outlier");
            if(temp_mod->active())
            {
                m_frame_log << "Valid update (Alpha = " << temp_mod->alpha() << ")";
                models_added++;
            }
            else
            {
                m_frame_log << "Outlier";
            }
            m_frame_log << std::endl;
#endif
        }
        removeInactiveModels(new_models);
        if(models_added)
        {
            (*model_it)->setActive(false);
        }
        else
        {
            (*model_it)->setAlpha(outlier_factor * (*model_it)->alpha());
        }
    }
    if(new_models.size() > 0)
    {
        m_models.insert(m_models.end(), new_models.begin(), new_models.end());
        new_models.clear();
    }
    return SelfModel::RESULT_OUTLIER;
}

int SelfLocalisation::ambiguousLandmarkUpdateSelective(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    const float c_Max_Elapsed_Time = 1.0f;

    // First step: Check if the same ambiguous type has beem seen recently.
    float time_elapsed = ambiguousObject.TimeLastSeen() - m_pastAmbiguous[ambiguousObject.getID()].TimeLastSeen();
    bool seen_recently = (time_elapsed < c_Max_Elapsed_Time);
    bool similar_meas_found = false;
    if(seen_recently)
    {
        // Second step: Check if the previous measurment was consistent with the current measurment

        // Calculate the distances between the two measurments
        float flatDistancePrev = m_pastAmbiguous[ambiguousObject.getID()].measuredDistance() * cos(m_pastAmbiguous[ambiguousObject.getID()].measuredElevation());
        float flatDistanceCurr = ambiguousObject.measuredDistance() * cos(ambiguousObject.measuredElevation());
        float dist_delta = fabs(flatDistanceCurr - flatDistancePrev);
        float heading_delta = fabs(ambiguousObject.measuredBearing() - m_pastAmbiguous[ambiguousObject.getID()].measuredBearing());

        // Calculate the allowable deviation
        MeasurementError error = calculateError(ambiguousObject);
        float max_distance_deviation = error.distance();
        float max_heading_deviation = error.heading();

        m_pastAmbiguous[ambiguousObject.getID()] = ambiguousObject;

        similar_meas_found = (dist_delta < max_distance_deviation) && (heading_delta < max_heading_deviation);
        if(similar_meas_found)
        {
            // Third Step (A): Apply mesurment using previous decision if measurement is consistant.
            ModelContainer new_models;
            SelfModel* temp_model = NULL;

            for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
            {
                // Get the id of the object to use for the update.
                unsigned int object_id = (*model_it)->previousSplitOption(ambiguousObject);
                // Find the option that matches the previous decision
                for(std::vector<StationaryObject*>::const_iterator obj_it = possibleObjects.begin(); obj_it != possibleObjects.end(); ++obj_it)
                {
                    if((*obj_it)->getID() == object_id)
                    {
                        // Perform the update
                        StationaryObject update_object(*(*obj_it));
                        update_object.CopyMeasurement(ambiguousObject);
                        (*model_it)->MeasurementUpdate(update_object, error);
                        new_models.push_back(new Model(*(*model_it)));  // Copy the new model and add it to the new list.
                        (*model_it)->setActive(false);
                        continue;
                    }
                }
                // Check if model has been updated yet
                // If not need to do a exhaustive update for this model
                if((*model_it)->active())
                {
                    bool update_performed = false;
                    for(vector<StationaryObject*>::const_iterator option_it = possibleObjects.begin(); option_it != possibleObjects.end(); ++option_it)
                    {
                        temp_model = new Model(*(*model_it), ambiguousObject, *(*option_it), error, m_timestamp);
                        if(temp_model->active())
                        {
                            new_models.push_back(temp_model);
                            update_performed = true;
                        }
                    }
                    if(update_performed)
                    {
                        (*model_it)->setActive(false);
                    }
                }
            }
            removeInactiveModels();
            m_models.insert(m_models.begin(),new_models.begin(), new_models.end());
        }
    }
    if(!similar_meas_found or !similar_meas_found)
    {
        // Third Step (B): Perfrom splitting if measurement was not previously seen or not consistant.
        // Use the regular splitting method.
        return ambiguousLandmarkUpdateExhaustive(ambiguousObject, possibleObjects);
    }
    return SelfModel::RESULT_OUTLIER;
}

int SelfLocalisation::ambiguousLandmarkUpdateProbDataAssoc(AmbiguousObject &ambiguousObject, const vector<StationaryObject*>& possibleObjects)
{
    MeasurementError error = calculateError(ambiguousObject);
    SelfModel::updateResult result = SelfModel::RESULT_FAILED;
    //result = m_models.front()->MeasurementUpdate(ambiguousObject, possibleObjects, error);
    return result;
}

/*! @brief Merges two models into a single model.
    @param modelA First model to merge.
    @param modelB Second model to merge.
    @return True if the models were merged successfully. False if unsuccessful.
*/
bool SelfLocalisation::MergeTwoModels(SelfModel* modelA, SelfModel* modelB)
{
    // Merges second model into first model, then disables second model.
    bool success = true;
    if(modelA == modelB) success = false; // Don't merge the same model.
    if(modelA->inactive() || modelB->inactive()) success = false; // Both models must be active.

    if(success == false)
    {
#if DEBUG_LOCALISATION_VERBOSITY > 2
        cout << "Merge failed." <<std::endl;
        debug_out  <<"[" << m_timestamp << "]: Merge Between model[" << modelA->id() << "] and model[" << modelB->id() << "] FAILED." << endl;
#endif // DEBUG_LOCALISATION_VERBOSITY > 0
        return success;
    }

    // Merge alphas
    double alphaMerged = modelA->alpha() + modelB->alpha();
    double alphaA = modelA->alpha() / alphaMerged;
    double alphaB = modelB->alpha() / alphaMerged;

    Matrix xMerged; // Merge State matrix

    // If one model is much more correct than the other, use the correct states.
    // This prevents drifting from continuouse splitting and merging even when one model is much more likely.
    if(alphaA > 10*alphaB)
    {
        xMerged = modelA->mean();
    } 
    else if (alphaB > 10*alphaA)
    {
        xMerged = modelB->mean();
    } 
    else
    {
        xMerged = (alphaA * modelA->mean() + alphaB * modelB->mean());
        // Fix angle.
        double angleDiff = modelB->mean(Model::states_heading) - modelA->mean(Model::states_heading);
        angleDiff = normaliseAngle(angleDiff);
        xMerged[Model::states_heading][0] = normaliseAngle(modelA->mean(Model::states_heading) + alphaB*angleDiff);
    }
 
    // Merge Covariance matrix (S = sqrt(P))
    Matrix xDiff = modelA->mean() - xMerged;
    Matrix pA = (modelA->covariance() + xDiff * xDiff.transp());

    xDiff = modelB->mean() - xMerged;
    Matrix pB = (modelB->covariance() + xDiff * xDiff.transp());
  
    Matrix pMerged = alphaA * pA + alphaB * pB;

    // Copy merged value to first model
    modelA->setAlpha(alphaMerged);
    modelA->setMean(xMerged);
    modelA->setCovariance(pMerged);

    // Disable second model
    modelA->setActive(true);
    modelB->setActive(false);
    return true;
}

/*! @brief Determines the number of active models.
    @returnThe number of models active.
*/
unsigned int SelfLocalisation::getNumActiveModels()
{
    unsigned int numActive = 0;
    for (ModelContainer::const_iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if((*model_it)->active() == true) numActive++;
    }
    return numActive;
}

/*! @brief Return the number of free models.
    @return The number of models available.
*/
unsigned int SelfLocalisation::getNumFreeModels()
{
    return c_MAX_MODELS - getNumActiveModels();
}

/*! @brief Comparison function for two models.
    @param modelA The first model.
    @param modelB The second model.
    @return True if modelA is smaller than modelB. False if Model A is greater than or equal to ModelB.
*/
bool model_alpha_lesser(SelfModel* modelA, SelfModel* modelB)
{
    return modelA->alpha() < modelB->alpha();
}

/*! @brief Retrieve the best available model.
    @return A pointer to the best available model.
*/
const SelfModel* SelfLocalisation::getBestModel() const
{
    assert(m_models.size() > 0);
    SelfModel* result;
    ModelContainer::const_iterator best_mod_it;

    best_mod_it = max_element(m_models.begin(), m_models.end(), model_alpha_lesser);
    result = (*best_mod_it);
    return result;
}

bool SelfLocalisation::CheckModelForOutlierReset(int modelID)
{
    // RHM 7/7/08: Suggested incorporation of 'Resetting' for possibly 'kidnapped' robot
    //----------------------------------------------------------
    double sum = 0.0;
    int numObjects = 0;
    bool reset = false;
    for(int objID = 0; objID < c_numOutlierTrackedObjects; objID++)
    {
        switch(objID)
        {
            case FieldObjects::FO_BLUE_LEFT_GOALPOST:
            case FieldObjects::FO_BLUE_RIGHT_GOALPOST:
            case FieldObjects::FO_YELLOW_LEFT_GOALPOST:
            case FieldObjects::FO_YELLOW_RIGHT_GOALPOST:
            case FieldObjects::FO_CORNER_BLUE_PEN_LEFT:
            case FieldObjects::FO_CORNER_BLUE_PEN_RIGHT:
            case FieldObjects::FO_CORNER_BLUE_T_LEFT:
            case FieldObjects::FO_CORNER_BLUE_T_RIGHT:
                sum += m_modelObjectErrors[modelID][objID];
                if (m_modelObjectErrors[modelID][objID] > c_OBJECT_ERROR_THRESHOLD) numObjects+=1;
                m_modelObjectErrors[modelID][objID] *= c_OBJECT_ERROR_DECAY;
                break;
            default:
                break;
        }
    }

    // Check if enough recent 'outliers' that we should reset ?
    if ((sum > c_RESET_SUM_THRESHOLD) && (numObjects >= c_RESET_NUM_THRESHOLD)) {
        reset = true;
        //m_models[modelID].Reset(); //Reset KF varainces. Leave Xhat!

        for (int i=0; i<c_numOutlierTrackedObjects; i++) m_modelObjectErrors[modelID][i] = 0.0; // Reset the outlier history
    }
    return reset;
}


/*! @brief Not yet implemented.
*/
int  SelfLocalisation::CheckForOutlierResets()
{
    int numResets = 0;
    return numResets;
}

/*! @brief Normalises the alphas of all exisiting models.
    The alphas of all existing models are normalised so that the total probablility of the set sums to 1.0.
*/
void SelfLocalisation::NormaliseAlphas()
{
    // Normalise all of the models alpha values such that all active models sum to 1.0
    double sumAlpha=0.0;
    for (ModelContainer::iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if ((*model_it)->active())
        {
            sumAlpha += (*model_it)->alpha();
        }
    }
    if(sumAlpha == 1) return;
    if (sumAlpha == 0) sumAlpha = 1e-12;
    for (ModelContainer::iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
        if ((*model_it)->active())
        {
            (*model_it)->setAlpha((*model_it)->alpha()/sumAlpha);
        }
    }
}

//**************************************************************************
//  This method begins the process of merging close models together
/*! @brief Merges models until the number active is below the maximum given.
    @param maxAfterMerge The maximum number of models that will remain after the mreging process.
*/
void SelfLocalisation::MergeModels(int maxAfterMerge)
{
    MergeModelsBelowThreshold(0.001);
    MergeModelsBelowThreshold(0.01);
  
//  double threshold=0.04;
    double threshold=0.05;

    while (getNumActiveModels()>maxAfterMerge)
    {
        MergeModelsBelowThreshold(threshold);
//      threshold*=5.0;
        threshold+=0.05;
    }
    removeInactiveModels();
    return;
}


/*! @brief Creates a summary of the localisation in human readable string form.

    @return A string containing the summary.
*/
std::string SelfLocalisation::ModelStatusSummary()
{
    std::stringstream temp;
    temp << "Active model summary." << std::endl;
    for (ModelContainer::iterator model_it = m_models.begin(); model_it != m_models.end(); ++model_it)
    {
            //! TODO: MISSING MEMBER FUNCTION
            temp << (*model_it)->summary();
    }
    return temp.str();
}

/*! @brief Prints a models status.

    @param model The model
*/
void SelfLocalisation::PrintModelStatus(const SelfModel* model)
{
#if DEBUG_LOCALISATION_VERBOSITY > 2
  debug_out  <<"[" << m_currentFrameNumber << "]: Model[" << model->id() << "]";
  debug_out  << "[alpha=" << model->alpha() << "]";
  debug_out  << " active = " << model->active() << endl;
#endif
  return;
}

/*! @brief Merges all model pairs with a merge metric below the given threshold.

    @param MergeMetricThreshold The threshold.
    @return The merge metric of the pair.
*/
void SelfLocalisation::MergeModelsBelowThreshold(double MergeMetricThreshold)
{
    double mergeM;
    for (ModelContainer::iterator i = m_models.begin(); i != m_models.end(); ++i) {
        for (ModelContainer::iterator j = i; j != m_models.end(); ++j) {
            if((*i) == (*j))
            {
                continue;
            }
            SelfModel* modelA = (*i);
            SelfModel* modelB = (*j);
            if (modelA->inactive() || modelB->inactive())
            {
                continue;
            }
            mergeM = abs( MergeMetric(modelA,modelB) );
            if (mergeM < MergeMetricThreshold) { //0.5
#if LOC_SUMMARY > 0
            m_frame_log << "Model " << (*j)->id() << " merged into " << (*i)->id() << std::endl;
#endif
#if DEBUG_LOCALISATION_VERBOSITY > 2
                debug_out  <<"[" << m_currentFrameNumber << "]: Merging Model[" << modelB->id() << "][alpha=" << modelB->alpha() << "]";
                debug_out  << " into Model[" << modelA->id() << "][alpha=" << modelA->alpha() << "] " << " Merge Metric = " << mergeM << endl  ;
#endif
                MergeTwoModels(modelA,modelB);
            }
        }
    }
}



//************************************************************************
// model to compute a metric for how 'far' apart two models are in terms of merging.
/*! @brief Merge metric calculation
    Calculates the merge metric of a pair of models.
    @param modelA The first model.
    @param ModelB The sencond model.

    @return The merge metric of the pair.
*/
double SelfLocalisation::MergeMetric(const SelfModel *modelA, const SelfModel *modelB) const
{   
    if (modelA==modelB) return 10000.0;
    if (modelA->inactive() || modelB->inactive()) return 10000.0; //at least one model inactive
    Matrix xdif = modelA->mean() - modelB->mean();
    Matrix p1 = modelA->covariance();
    Matrix p2 = modelB->covariance();
  
    xdif[Model::states_heading][0] = normaliseAngle(xdif[Model::states_heading][0]);

    double dij=0;
    for (int i=0; i<p1.getm(); i++) {
        dij+=(xdif[i][0]*xdif[i][0]) / (p1[i][i]+p2[i][i]);
    }
    return dij*( (modelA->alpha()*modelB->alpha()) / (modelA->alpha()+modelB->alpha()) );
}

/*! @brief output stream
    Writes the localisation object to an output stream
    @param output The output stream to which the data will be written
    @param p_loc The localisation to be writen.
    @return The ouput stream
*/
std::ostream& operator<< (std::ostream& output, const SelfLocalisation& p_loc)
{    
    output.write(reinterpret_cast<const char*>(&p_loc.m_timestamp), sizeof(p_loc.m_timestamp));
    output.write(reinterpret_cast<const char*>(&p_loc.m_currentFrameNumber), sizeof(p_loc.m_currentFrameNumber));
    output.write(reinterpret_cast<const char*>(&p_loc.m_previously_incapacitated), sizeof(p_loc.m_previously_incapacitated));
    output.write(reinterpret_cast<const char*>(&p_loc.m_previous_game_state), sizeof(p_loc.m_previous_game_state));
    output.write(reinterpret_cast<const char*>(&p_loc.m_hasGps), sizeof(p_loc.m_hasGps));
    if(p_loc.m_hasGps)
    {
        output.write(reinterpret_cast<const char*>(&p_loc.m_compass), sizeof(p_loc.m_compass));
        output.write(reinterpret_cast<const char*>(&p_loc.m_gps[0]), sizeof(p_loc.m_gps[0]));
        output.write(reinterpret_cast<const char*>(&p_loc.m_gps[1]), sizeof(p_loc.m_gps[1]));
    }

    unsigned int num_models = p_loc.m_models.size();
    output.write(reinterpret_cast<const char*>(&num_models), sizeof(num_models));
    for (ModelContainer::const_iterator model_it = p_loc.m_models.begin(); model_it != p_loc.m_models.end(); ++model_it)
    {
        output << *(*model_it);
    }

    return output;
}

/*! @brief input stream
    Reads in the localisation from an input stream.
    @param input The input stream fro which data will be read.
    @param p_loc The localisation to which the data will be written.
    @return The input stream
*/
std::istream& operator>> (std::istream& input, SelfLocalisation& p_loc)
{
    input.read(reinterpret_cast<char*>(&p_loc.m_timestamp), sizeof(p_loc.m_timestamp));
    input.read(reinterpret_cast<char*>(&p_loc.m_currentFrameNumber), sizeof(p_loc.m_currentFrameNumber));
    input.read(reinterpret_cast<char*>(&p_loc.m_previously_incapacitated), sizeof(p_loc.m_previously_incapacitated));
    input.read(reinterpret_cast<char*>(&p_loc.m_previous_game_state), sizeof(p_loc.m_previous_game_state));
    input.read(reinterpret_cast<char*>(&p_loc.m_hasGps), sizeof(p_loc.m_hasGps));
    if(p_loc.m_hasGps)
    {
        input.read(reinterpret_cast<char*>(&p_loc.m_compass), sizeof(p_loc.m_compass));
        input.read(reinterpret_cast<char*>(&p_loc.m_gps[0]), sizeof(p_loc.m_gps[0]));
        input.read(reinterpret_cast<char*>(&p_loc.m_gps[1]), sizeof(p_loc.m_gps[1]));
    }

    p_loc.clearModels();

    unsigned int num_models;
    input.read(reinterpret_cast<char*>(&num_models), sizeof(num_models));
    p_loc.m_models.resize(num_models);

    for (ModelContainer::iterator model_it = p_loc.m_models.begin(); model_it != p_loc.m_models.end(); ++model_it)
    {
        (*model_it) = new Model(p_loc.m_timestamp);
        input >> *(*model_it);
    }
    return input;
}

/*! @brief Clears all of the current models.

 */
void SelfLocalisation::clearModels()
{
    for (ModelContainer::iterator mod = m_models.begin(); mod != m_models.end(); mod++)
    {
        delete *mod;
        *mod = 0;
    }
    m_models.clear();
}

/*! @brief Initialises all of the models desribed in the vector and weights them evenly.

    @param positions A vector containing any number of moments (measurements) of position.
*/
void SelfLocalisation::InitialiseModels(const std::vector<Moment>& positions)
{
    if(positions.size() <= 0) return;      // Don't do anything if no positions are given.
    const float split_alpha = 1.0f / positions.size();            // Uniform split alpha for all models.
    SelfModel* temp;
    clearModels();

#if LOC_SUMMARY > 0
    m_frame_log << "Intitialising models." << std::endl;
    m_frame_log << "Positions:" << std::endl;
    for (std::vector<Moment>::const_iterator pos_it = positions.begin(); pos_it != positions.end(); ++pos_it)
    {
        m_frame_log << "(" << (*pos_it).mean(Model::states_x) << "," << (*pos_it).mean(Model::states_y) << "," << (*pos_it).mean(Model::states_heading);
        m_frame_log << ") - (" << (*pos_it).sd(Model::states_x) << "," << (*pos_it).sd(Model::states_y) << "," << (*pos_it).sd(Model::states_heading) << std::endl;
    }
#endif

    for (std::vector<Moment>::const_iterator pos = positions.begin(); pos != positions.end(); pos++)
    {
        temp = new Model(GetTimestamp());

        temp->setAlpha(split_alpha);
        temp->setMean((*pos).mean());
        temp->setCovariance((*pos).covariance());
        temp->setActive(true);
        m_models.push_back(temp);
    }
    return;
}


void SelfLocalisation::setModels(ModelContainer& newModels)
{
    clearModels();
    m_models = newModels;
    return;
}

/*! @brief Create a 3x1 mean matrix with value as defiend by the parameters.

Creates a 3x1 matrix with the mean of each attribute set as specified.

@param x_var Mean in the x state.
@param y_var Mean in the y state.
@param heading_var Mean in the heading state.
@retun A matrix implementation of the described mean vector.
*/
Matrix SelfLocalisation::mean_matrix(float x, float y, float heading)
{
    Matrix temp_mean(Model::states_total, 1, false);
    temp_mean[Model::states_x][0] = x;
    temp_mean[Model::states_y][0] = y;
    temp_mean[Model::states_heading][0] = heading;
    return temp_mean;
}

/*! @brief Create a 3x3 covariance matrix with diagonals as defiend by the parameters.

Creates a 3x3 covariance matrix with the variance of each attribute set as specified.

@param x_var Variance in the x state.
@param y_var Variance in the y state.
@param heading_var Variance in the heading state.
@retun A matrix implementation of the described covariance.
*/
Matrix SelfLocalisation::covariance_matrix(float x_var, float y_var, float heading_var)
{
    Matrix temp_cov(SelfModel::states_total, SelfModel::states_total, false);
    temp_cov[Model::states_x][Model::states_x] = x_var;
    temp_cov[Model::states_y][Model::states_y] = y_var;
    temp_cov[Model::states_heading][Model::states_heading] = heading_var;
    return temp_cov;
}


/*! @brief Remove Inactive model from the default container

@retun The number of models removed.
*/
unsigned int SelfLocalisation::removeInactiveModels()
{
    unsigned int result = removeInactiveModels(m_models);  // Remove inactive models from the default container.
    unsigned int numActive = getNumActiveModels();
    assert(m_models.size() == numActive);
    return result;
}

bool is_null(const SelfModel* pointer)
{
    return (pointer == NULL);
}

/*! @brief Remove Inactive model from a specified container.

Iterates through the container and removes any inactive models found.

@param container The container to remove inactive models from.
@retun The number of models removed.
*/
unsigned int SelfLocalisation::removeInactiveModels(ModelContainer& container)
{
    const unsigned int num_before = container.size();   // Save original size

    for (ModelContainer::iterator model_it = container.begin(); model_it != container.end(); ++model_it)
    {
        if((*model_it)->inactive())
        {
            delete (*model_it);
            (*model_it) = NULL;
        }
    }
    container.erase(remove_if(container.begin(), container.end(), is_null), container.end());
    return num_before - container.size();               // Return number removed, original size - new size
}

/*!
@brief Calculate the variance in the given measurement based on the objects distance.
@param theObject The object the error variance is to be calculated for.
@return The error of the measurement given as the variance.
*/
MeasurementError SelfLocalisation::calculateError(const Object& theObject)
{
    MeasurementError error;
    error.setDistance(c_obj_range_offset_variance + c_obj_range_relative_variance * pow(theObject.measuredDistance() * cos(theObject.measuredElevation()),2));
    error.setHeading(c_obj_theta_variance);
    return error;
}
