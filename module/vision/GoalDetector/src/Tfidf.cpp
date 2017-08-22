#include "Tfidf.h"
#include <stdio.h>
#include <iostream>
#include "RANSACLine.h"
#include "Ransac.h"


// Loads vocab ready for use
void Tfidf::loadVocab(std::string vocabFile) {

    // clear everything
    clearData();

    // load the vocab file
    vocab.loadVocabFile(vocabFile);
    T  = vocab.getSize();
    ni = Eigen::VectorXf::Zero(T);
    printf("Loaded vocab of %d words for GoalMatcher\n", T);
}

int Tfidf::getSize() {
    return N;
}

void Tfidf::clearMap() {
    clearData();
    T  = vocab.getSize();
    ni = Eigen::VectorXf::Zero(T);
}

float Tfidf::getValidCosineScore() {
    return VALID_COSINE_SCORE;
}

int Tfidf::getValidInliers() {
    return VALID_INLIERS;
}

void Tfidf::setValidCosineScore(float x) {
    VALID_COSINE_SCORE = x;
}

void Tfidf::setValidInliers(int x) {
    VALID_INLIERS = x;
}

// Loads map ready for use (needs a vocab first)

void Tfidf::loadMap(std::string mapFile) {

    if (T != 0) {
        // clear previous map
        clearMap();

        // load the map file
        std::vector<MapEntry> tempMap;
        std::ifstream ifs(mapFile.c_str());
        if (ifs.is_open()) {
            boost::archive::text_iarchive ia(ifs);
            ia >> tempMap;
        }
        else {
            throw std::runtime_error("error opening map file in tfidf");
        }

        for (int i = 0; i < (int) tempMap.size(); i++) {
            addDocumentToCorpus(tempMap.at(i));
        }
        printf("Loaded map with %d entries\n", map.size());
    }
}

// Saves map, including any new entries
void Tfidf::saveMap(std::string mapFile) {

    std::ofstream ofs(mapFile.c_str());
    {
        boost::archive::text_oarchive oa(ofs);
        oa << map;
    }
}

//! Adds to the searchable collection, return true if successful
bool Tfidf::addDocumentToCorpus(MapEntry document) {
    if (vocab.getSize() != 0) {
        std::unique_ptr<std::vector<std::vector<float>>> pixLoc = std::make_unique<std::vector<std::vector<float>>>();
        std::unique_ptr<std::vector<Ipoint>> ipoints_ptr        = std::make_unique<std::vector<Ipoint>>();
        *ipoints_ptr                                            = document.ipoints;
        Eigen::VectorXf tf_doc                                  = vocab.mapToVec(ipoints_ptr, pixLoc);
        return addDocumentToCorpus(document, tf_doc, *pixLoc);
    }
    return false;
}

//! Faster version if landmarks have already been mapped to words (with the same vocab file)
bool Tfidf::addDocumentToCorpus(MapEntry document, Eigen::VectorXf tf_doc, std::vector<std::vector<float>> pixLoc) {

    bool result = false;
    printf("addDocumentToCorpus:\n");
    if (vocab.getSize() != 0) {
        printf("vocab size criteria passed.\n");
        if (tf_doc.sum() != 0) {  // don't let an empty document be added
            printf("Adding tf_doc now (tf_doc sum = %f\n", tf_doc.sum());
            /*
            printf("tf_doc = [");
            int count = 0;
            for (int j = 0; j < tf_doc.size();j++){
                if (tf_doc[j] < 0.001){
                    count++;
                }
                else {
                    if (count > 0) printf("...//%d//...",count);
                    printf("(%.2f)",tf_doc[j]);
                    count = 0;
                }
            }
            if (count > 0) printf("...//%d//...",count);
            printf("]\n\n");
            */
            tf.push_back(tf_doc);
            pixels.push_back(pixLoc);
            ni = ni + tf_doc;
            N++;
            nd.push_back(tf_doc.sum());
            map.push_back(document);
            //! Recalculate the inverse document frequency (log(N/ni))
            idf = (ni.array().inverse() * N).log();
            // Remove Inf from idf (can occur with random initialised learned words)
            for (int i = 0; i < T; i++) {
                if (idf[i] == std::numeric_limits<float>::infinity()) {
                    idf[i] = 0.f;
                }
            }
            /*
            printf("idf = [");
            count = 0;
            for (int j = 0; j < idf.size();j++){
                if (idf[j] > 88.0){
                    count++;
                }
                else {
                    if (count > 0) printf("...//%d//...",count);
                    printf("(%.2f)",idf[j]);
                    count = 0;
                }
            }
            if (count > 0) printf("...//%d//...",count);
            printf("]\n\n");
            */
            result = true;
        }
        else {
            printf("tf_doc was an empty document\n");
        }
    }
    else {
        printf("vocab size criteria FAILED\n");
    }
    return result;
}

//! Faster version if landmarks have already been mapped to words
void Tfidf::searchDocument(Eigen::VectorXf tf_query,
                           std::vector<std::vector<float>> query_pixLoc,  // pixel locations of the words
                           std::unique_ptr<std::priority_queue<MapEntry>>& matches,
                           unsigned int* seed,
                           int num,
                           Eigen::MatrixXd* resultTable,
                           int queryGoalWidth) {

    NUClear::clock::time_point t = NUClear::clock::now();
    printf("tf_query.sum = %.1f, N = %d\n", tf_query.sum(), N);
    if (tf_query.sum() != 0 && N != 0) {  // checked the document is not empty and corpus not empty
        printf("Running Tfidf::searchDocument now.\n");
        std::priority_queue<std::pair<MapEntry, std::pair<std::vector<std::vector<float>>, int>>> queue;
        Eigen::VectorXf tfidf_query = (tf_query / tf_query.sum()).array() * idf.array();

        int count = 0;
        /*
        printf("tf_query =    [");
        for (int j = 0; j < tf_query.size();j++){
            if (tf_query[j] < 0.001){
                count++;
            }
            else {
                if (count > 0) printf("...//%d//...",count);
                printf("(%.2f)",tf_query[j]);
                count = 0;
            }
        }
        if (count > 0) printf("...//%d//...",count);
        printf("]\n\n");
        */
        /*
        printf("tfidf_query = [");
        count = 0;
        for (int j = 0; j < tfidf_query.size();j++){
            if (tfidf_query[j] < 0.001){
                count++;
            }
            else {
                if (count > 0) printf("...//%d//...",count);
                printf("(%.2f)",tfidf_query[j]);
                count = 0;
            }
        }
        if (count > 0) printf("...//%d//...",count);
        printf("]\n\n");
        */

        // Now compute the cosines against each document -- inverted index not used since our vectors are not sparse
        // enough
        Eigen::VectorXf tfidf_doc;
        printf("Cosine scores: (for N=%d)\n", N);
        for (int i = 0; i < N; i++) {
            tfidf_doc = (tf[i] / nd[i]).array() * idf.array();  // nd[i] can't be zero or it wouldn't have been added

            // printf("nd[i] = %d\n",nd[i]);
            /*
            printf("tfidf_doc = [");
            int count = 0;
            for (int j = 0; j < tfidf_doc.size();j++){
                if (tfidf_doc[j] < 0.001){
                    count++;
                }
                else {
                    if (count > 0) printf("...//%d//...",count);
                    printf("(%.2f)",tfidf_doc[j]);
                    count = 0;
                }
            }
            if (count > 0) printf("...//%d//...",count);
            printf("]\n");
            */
            // printf("tfidf_query dot tfidf_doc : %f, tfidf_query norm: %.2f, tfidf_doc norm:
            // %.2f\n",tfidf_query.dot(tfidf_doc),tfidf_query.norm(),tfidf_doc.norm());

            map.at(i).score = cosineScore(tfidf_query, tfidf_doc);
            printf("%2d. map score: %.2f   ", i + 1, map[i].score);
            if (map.at(i).score > VALID_COSINE_SCORE) {
                printf("This is a valid cosine score");
                queue.push(std::make_pair(map.at(i), std::make_pair(pixels.at(i), goalWidth.at(i))));
                /*
                printf("tfidf_doc = [");
                int count = 0;
                for (int j = 0; j < tfidf_doc.size();j++){
                    if (tfidf_doc[j] < 0.001){
                        count++;
                    }
                    else {
                        if (count > 0) printf("...//%d//...",count);
                        printf("(%.2f)",tfidf_doc[j]);
                        count = 0;
                    }
                }
                if (count > 0) printf("...//%d//...",count);
                printf("]\n");
                */
            }
            printf("\n");
        }
        printf("Complete.\n");

        auto start = std::chrono::system_clock::now();
        // Now do geometric validation on the best until we have enough or the queue is empty
        int counter = -1;
        while (!queue.empty() && matches->size() < (unsigned int) num) {
            MapEntry mapEntry = queue.top().first;
            printf("Validating Cos: %.2f ", mapEntry.score);

            counter++;
            (*resultTable)(counter, 0) = mapEntry.score;
            if ((mapEntry.position.theta() < M_PI / 2) && (mapEntry.position.theta() > -M_PI / 2)) {
                printf("(OPP)");
                (*resultTable)(counter, 1) = 1.0;
                (*resultTable)(0, 4)       = (*resultTable)(0, 4) + 1.0;
            }
            else {
                printf("(OWN)");
                (*resultTable)(counter, 1) = -1.0;
                (*resultTable)(0, 5)       = (*resultTable)(0, 5) + 1.0;
            }

            std::vector<std::vector<float>> pixLoc = queue.top().second.first;
            int storedGoalWidth                    = queue.top().second.second;
            queue.pop();

            auto temp_start1 = std::chrono::system_clock::now();
            // Do average xpos
            // Average pixel location for stored matches (should be precalculated to save time)
            std::vector<float> pixLocAvg(pixLoc.size(), 0.0);
            for (int m = 0; m < pixLoc.size(); m++) {  // stepping along each feature
                for (int n = 0; n < pixLoc.at(m).size(); n++) {
                    pixLocAvg.at(m) += pixLoc.at(m).at(n);
                }
                if (pixLoc.at(m).size() >= 1) pixLocAvg.at(m) /= pixLoc.at(m).size();
            }

            // Average pixel location for query features
            std::vector<float> query_pixLocAvg(query_pixLoc.size(), 0);
            for (int m = 0; m < query_pixLoc.size(); m++) {  // stepping along each feature
                for (int n = 0; n < query_pixLoc.at(m).size(); n++) {
                    query_pixLocAvg.at(m) += query_pixLoc.at(m).at(n);
                }
                if (query_pixLoc.at(m).size() >= 1) query_pixLocAvg.at(m) /= query_pixLoc.at(m).size();
            }
            auto temp_end1   = std::chrono::system_clock::now();
            auto temp_start2 = std::chrono::system_clock::now();

            // Creating 2 matricies of distances between features, one for match, one for query
            Eigen::MatrixXf pixLocDist;
            Eigen::MatrixXf query_pixLocDist;
            pixLocDist.setZero(pixLoc.size(), pixLoc.size());
            query_pixLocDist.setZero(query_pixLoc.size(), query_pixLoc.size());

            for (int m = 0; m < query_pixLoc.size(); m++) {
                for (int n = m + 1; n < query_pixLoc.size(); n++) {
                    if ((pixLoc.at(m).size() >= 1) && (pixLoc.at(n).size() >= 1)) {
                        pixLocDist(m, n) = fabs(pixLocAvg.at(n) - pixLocAvg.at(m));
                    }
                    if ((query_pixLoc.at(m).size() >= 1) && (query_pixLoc.at(n).size() >= 1)) {
                        query_pixLocDist(m, n) = fabs(query_pixLocAvg.at(n) - query_pixLocAvg.at(m));
                    }
                }
            }
            auto temp_end2   = std::chrono::system_clock::now();
            auto temp_start3 = std::chrono::system_clock::now();
            // Comparing the two distance matricies
            float scalingFactor = (float) queryGoalWidth / storedGoalWidth;
            Eigen::MatrixXf pixLocDiff;
            pixLocDiff = (query_pixLocDist - scalingFactor * pixLocDist).array().abs();
            for (int m = 0; m < query_pixLoc.size(); m++) {
                if ((pixLoc.at(m).size() < 1)
                    && (query_pixLoc.at(m).size() < 1)) {  // Removing distances for features neither have
                    pixLocDiff.row(m).setZero();
                    pixLocDiff.col(m).setZero();
                }
                else if ((pixLoc.at(m).size() < 1) && (query_pixLoc.at(m).size() >= 1)) {
                    pixLocDiff.row(m).setZero();
                    pixLocDiff.col(m).setZero();
                }
                else if ((pixLoc.at(m).size() >= 1) && (query_pixLoc.at(m).size() < 1)) {
                    pixLocDiff.row(m).setZero();
                    pixLocDiff.col(m).setZero();
                }
            }

            float pixDistScore = pixLocDiff.sum();
            auto temp_end3     = std::chrono::system_clock::now();
            printf(" pixDistScore: %.0f, scaling factor: %.2f, ", pixDistScore, scalingFactor);
            auto temptimecheck1 = std::chrono::duration_cast<std::chrono::microseconds>(temp_end1 - temp_start1);
            auto temptimecheck2 = std::chrono::duration_cast<std::chrono::microseconds>(temp_end2 - temp_start2);
            auto temptimecheck3 = std::chrono::duration_cast<std::chrono::microseconds>(temp_end3 - temp_start3);
            std::cout << "Time to get avg xpos: " << temptimecheck1.count() << " ms, ";
            std::cout << "Time to get internal xpos diffs: " << temptimecheck2.count() << " ms, ";
            std::cout << "Time to get diff score: " << temptimecheck3.count() << " ms";

            // /*
            // Do RANSAC geometric validation - first build the points to run ransac
            std::vector<Point> matchpoints;
            for (int j = 0; j < T; j++) {
                for (uint32_t m = 0; m < pixLoc[j].size(); m++) {
                    for (uint32_t n = 0; n < query_pixLoc[j].size(); n++) {
                        matchpoints.push_back(Point(pixLoc[j][m], query_pixLoc[j][n]));
                        // printf("matchpoints: %.1f, %.1f\n",pixLoc[j][m], query_pixLoc[j][n]);
                    }
                }
            }

            // ransac
            RANSACLine resultLine;
            uint16_t min_points = 2;
            std::vector<bool>*con, consBuf[2];
            consBuf[0].resize(matchpoints.size());
            consBuf[1].resize(matchpoints.size());
            float slopeConstraint = 2.f;  // reject scale changes more than twice or half the distance
            bool ransacresult     = RANSAC::findLineConstrained(matchpoints,
                                                            &con,
                                                            resultLine,
                                                            MATCH_ITERATIONS,
                                                            PIXEL_ERROR_MARGIN,
                                                            min_points,
                                                            consBuf,
                                                            seed,
                                                            slopeConstraint);

            // printf("RANSAC result = %d\n",ransacresult);
            // printf("result line = (%.2f,%.2f)(%.2f,%.2f)\n", resultLine.p1[0], resultLine.p1[1], resultLine.p2[0],
            // resultLine.p2[1]);
            if (ransacresult && (resultLine.t2 != 0.f)) {  // check t2 but should be fixed by slope constraint anyway
                // count the inliers
                int inliers = 0;
                for (int v = 0; v < (int) matchpoints.size(); v++) {
                    if ((*con)[v]) {
                        inliers++;
                    }
                }
                printf(", RANSAC found %d inliers, %d outliers, ", inliers, (int) matchpoints.size() - inliers);
                (*resultTable)(counter, 2) = (double) inliers;
                (*resultTable)(counter, 3) = (double) matchpoints.size() - (double) inliers;
                // printf("at (x,y,theta) loc: (%.1f, %.1f, %.1f)\n", mapEntry.position.x(), mapEntry.position.y(),
                // mapEntry.position.theta());

                if (inliers >= VALID_INLIERS) {
                    printf("Location is valid\n");
                    matches->push(mapEntry);
                }
                else {
                    printf("Location INVALID\n");
                    num--;
                    if ((*resultTable)(counter, 1) > 0.5) (*resultTable)(0, 4)  = (*resultTable)(0, 4) - 1.0;
                    if ((*resultTable)(counter, 1) < -0.5) (*resultTable)(0, 5) = (*resultTable)(0, 5) - 1.0;
                }
            }
            else {
                // printf("SKIPPED INLIER COUNT SINCE ransacresult = %d\n",ransacresult);
                num--;
                if ((*resultTable)(counter, 1) > 0.5) (*resultTable)(0, 4)  = (*resultTable)(0, 4) - 1.0;
                if ((*resultTable)(counter, 1) < -0.5) (*resultTable)(0, 5) = (*resultTable)(0, 5) - 1.0;
            }
            //*/
        }
        auto end               = std::chrono::system_clock::now();
        auto spatialcheck_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Time to run RANSAC: " << spatialcheck_time.count() << " microseconds." << std::endl;
    }
    // printf("\nCalculating cosines over " << map.size() << " images, RANSAC geo validation & position adjustments took
    // ";
    // llog(DEBUG1) << t.elapsed_us() << " us" << std::endl;
}

float Tfidf::cosineScore(Eigen::VectorXf a, Eigen::VectorXf b) {
    return a.dot(b) / (std::max(double(a.norm() * b.norm()), 0.0000000000001));
}

float Tfidf::PearsonsCorrelation(Eigen::VectorXf a, Eigen::VectorXf b) {
    float sumAB  = (a.array() * b.array()).sum();
    float sumA   = a.array().sum();
    float sumB   = b.array().sum();
    float sumAsq = a.array().square().sum();
    float sumBsq = b.array().square().sum();
    float n      = a.size();

    return (sumAB - sumA * sumB / n) / (sqrt((sumAsq - sumA * sumA / n) * (sumBsq - sumB * sumB / n)));
}
