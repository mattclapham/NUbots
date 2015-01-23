/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include <yaml-cpp/yaml.h>

#include "messages/support/nubugger/proto/Message.pb.h"
#include "utility/time/time.h"
#include "utility/file/fileutil.h"
#include "utility/strutil/strutil.h"

/**
 * @author Monica Olejniczak
 */
namespace modules {
namespace support {
    using utility::file::listFiles;
    using utility::time::getUtcTimestamp;
    using utility::strutil::split;
    using messages::support::nubugger::proto::Message;
    using messages::support::nubugger::proto::ConfigurationState;

    void processNode(ConfigurationState::Node& node, YAML::Node& yaml);
    void processPath(std::string path, int currentIndex, ConfigurationState::Node& node, std::map<std::string,
            ConfigurationState::KeyPair*>& directories);
    void processConfiguration(std::string path, YAML::Node& root, YAML::Node& yaml, ConfigurationState::Node node);

    /**
     * @brief Processes a null YAML node by setting its respective type on the protocol node.
     *
     * @param node The protocol node.
     */
    void processNullNode(ConfigurationState::Node& node) {
        // set the type of the protocol node to the NULL_VALUE Node Type
        node.set_type(ConfigurationState::Node::NULL_VALUE);
    }

    /**
     * @brief Processes a scalar YAML node by setting its respective type on the protocol node. A scalar node may 
     * comprise a boolean, number or string.
     *
     * @param node The protocol node.
     * @param yaml A scalar YAML node.
     */
    void processScalarNode(ConfigurationState::Node& node, YAML::Node& yaml) {
        bool bValue;
        // check if the yaml node is a boolean
        if (YAML::convert<bool>::decode(yaml, bValue)) {
            // set the type of the protocol node to the BOOLEAN Node Type and escape the case
            node.set_type(ConfigurationState::Node::BOOLEAN);
            // set the value of the protocol node to the boolean
            node.set_boolean_value(bValue);
            return;
        }
        long lValue;
        // check if the yaml node is a long
        if (YAML::convert<long>::decode(yaml, lValue)) {
            // set the type of the protocol node to the LONG Node Type and escape the case
            node.set_type(ConfigurationState::Node::LONG);
            // set the value of the protocol node to the long
            node.set_long_value(lValue);
            return;
        }
        double dValue;
        // check if the yaml node is a double
        if (YAML::convert<double>::decode(yaml, dValue)) {
            // set the type of the protocol node to the DOUBLE Node Type and escape the case
            node.set_type(ConfigurationState::Node::DOUBLE);
            // set the value of the protocol node to the double
            node.set_double_value(dValue);
            return;
        }
        // declare a string and then decode it
        std::string sValue;
        YAML::convert<std::string>::decode(yaml, sValue);
        // set the type of the protocol node to the STRING Node Type and escape the case
        node.set_type(ConfigurationState::Node::STRING);
        // set the value of the protocol node to the string
        node.set_string_value(sValue);
    }

    /**
     * @brief Processes a sequence YAML node by settings its respective type on the protocol node. It then iterates 
     * through all the nodes in this sequence and processes each node.
     *
     * @param node The protocol node.
     * @param yaml A sequence YAML node.
     */
    void processSequenceNode(ConfigurationState::Node& node, YAML::Node& yaml) {
        // set the type of the protocol node to the SEQUENCE Node Type
        node.set_type(ConfigurationState::Node::SEQUENCE);
        // iterate through every yaml node in the sequence
        for (auto&& yamlNode : yaml) {
            // check if the node should be processed
            if (yamlNode.Tag() != NUbugger::IGNORE_TAG) {
                // recursively calls the function with a new map value
                processNode(*node.add_sequence_value(), yamlNode);
            }
        }
    }

    /**
     * @brief Processes a map YAML node by setting its respective type on the protocol node. It then iterates through 
     * all the nodes in this map and processes each node.
     *
     * @param node The protocol node.
     * @param yaml A map YAML node.
     */
    void processMapNode(ConfigurationState::Node& node, YAML::Node& yaml) {
        // set the type of the protocol node to the MAP Node Type
        node.set_type(ConfigurationState::Node::MAP);
        // iterate through every yaml node in the map
        for (auto&& yamlNode : yaml) {
            // check if the node should be processed
            if (yamlNode.second.Tag() != NUbugger::IGNORE_TAG) { 
                // create a new map value from the protocol node
                auto* map = node.add_map_value();
                // set the name of this new node to the key of the yaml node and convert it to a string
                map->set_name(yamlNode.first.as<std::string>());
                // recursively calls the function with a new map value
                processNode(*map->mutable_value(), yamlNode.second);
            }
        }
    }

    /**
     * @brief Processes a particular YAML node into its equivalent protocol node. The tag is initially set and the type of
     * the YAML node is then evaluated.
     *
     * @param node The protocol node.
     * @param yaml A YAML node.
     */
    void processNode(ConfigurationState::Node& node, YAML::Node& yaml) {
        // set the tag of the protocol buffer if it exists
        if (yaml.Tag() != "?") {
            node.set_tag(yaml.Tag());
        }
        switch (yaml.Type()) {
            case YAML::NodeType::Undefined:
                // TODO Handle it somehow?
                break;
            case YAML::NodeType::Null:
                processNullNode(node);
                break;
            // strings and numbers
            case YAML::NodeType::Scalar:
                processScalarNode(node, yaml);
                break;
            // arrays and lists
            case YAML::NodeType::Sequence:
                processSequenceNode(node, yaml);
                break;
            // hashes and dictionaries
            case YAML::NodeType::Map:
                processMapNode(node, yaml);
                break;
        }
    }

    /**
     * @brief Processes a configuration file by setting the type of the protocol node, loading the file through its path
     * and then processing the information within that file.
     *
     * @param path The path to the configuration file.
     * @param name The name of the configuration file.
     * @param node The protocol node.
     */
    void processFile(std::string path, std::string name, ConfigurationState::Node& node) {
        // create a new map value from the current protocol node
        auto* map = node.add_map_value();
        map->set_name(name);                                // set the name of the map
        map->set_path(path);                                // set the path of the map
        node.set_type(ConfigurationState::Node::FILE);      // set the Node Type of the map
        YAML::Node yaml = YAML::LoadFile(path);             // load the YAML file using the current iteration
        processNode(*map->mutable_value(), yaml);           // processes the yaml node into a protocol node
    }

    /**
     * @brief A directory is processed by first retrieving the path to it and determining if it exists in the directories 
     * map. If the directory is not found within this map, then a new directory node is created and added to it. Otherwise,
     * the directory node is accessed. Finally, the next section of the path is processed.
     *
     * @param path The path of the configuration file.
     * @param name The name of the directory.
     * @param node The protocol node.
     * @param index The index within the path that specifies where the name of the current directory ends.
     * @param directories The list of known directory nodes.
     */
    void processDirectory(std::string path, std::string name, ConfigurationState::Node& node, int index,
    std::map<std::string, ConfigurationState::KeyPair*>& directories) {
        // calculate the path to the directory
        std::string directoryPath = path.substr(0, index);
        // retrieve the directory protocol node given the path to the directory
        auto iterator = directories.find(directoryPath);
        ConfigurationState::KeyPair* map;                   // create the map key pair
        if (iterator == directories.end()) {                // check if the directory does not exist
            map = node.add_map_value();                     // create a new map value from the current protocol node
            map->set_name(name);                            // set the name of the map
            map->set_path(directoryPath);                   // set the path of the map
            // set the Node Type of the directory
            node.set_type(ConfigurationState::Node::DIRECTORY);
            // add the directory to the map
            directories[directoryPath] = map;
        } else {
            // get the map from the iterator
            map = iterator->second;
        }
        // process the new path
        processPath(path, index + 1, *map->mutable_value(), directories);
    }
    /**
     * @brief Processes a configuration file path recursively. The name of the file or directory is retrieved by 
     * using the index of the first known '/'. This index determines whether a file or directory needs to be processed.
     *
     * @param path The path of the configuration file.
     * @param currentIndex The current index to use when searching the path.
     * @param node The protocol node.
     * @param directories The list of known directory nodes.
     */
    void processPath(std::string path, int currentIndex, ConfigurationState::Node& node, std::map<std::string,
    ConfigurationState::KeyPair*>& directories) {
        // get the index of where the name of the file or directory ends
        auto index = path.find('/', currentIndex);
        // get the name of the current file or directory
        std::string name = path.substr(currentIndex, index - currentIndex);
        // check if we are at a file
        if (index == std::string::npos) {
            // process the file
            processFile(path, name, node);
        } else {
            // process the directory
            processDirectory(path, name, node, index, directories);
        }
    }

    /**
     * @brief Process a configuration file path by calling its recursive function.
     *
     * @param path The path of the configuration file.
     * @param node The protocol node.
     * @param directories The list of known directory nodes.
     */
    void processPath(std::string path, ConfigurationState::Node& node, std::map<std::string,
    ConfigurationState::KeyPair*>& directories) {
        processPath(path, 0, node, directories);
    }

    /**
     * @brief A command sent when the network requests the configuration state data. It retrieves all of the configuration
     * files and converts them into readable messages to send back over the network.
     */
    void NUbugger::sendConfigurationState() {
        // specify the base directory
        std::string directory = "config";
        // get the list of file paths in the shared config directory and ensure it is recursive
        std::vector<std::string> paths = listFiles(directory, true);
        // create the list of directories
        std::map<std::string, ConfigurationState::KeyPair*> directories;

        Message message;                                        // create a new message
        message.set_type(Message::CONFIGURATION_STATE);         // set the message type to the configuration state
        message.set_filter_id(0);                               // ensure the message is not filtered
        message.set_utc_timestamp(getUtcTimestamp());           // set the timestamp of the message

        auto* state = message.mutable_configuration_state();    // create the configuration state from the message
        auto* root = state->mutable_root();                     // retrieve the root node from the state
        root->set_type(ConfigurationState::Node::DIRECTORY);    // set the type of the root node to a directory

        for (auto&& path : paths) {                             // iterate through every file path in the config directory
            processPath(path, *root, directories);              // process the path using the root node
        }
        send(message);                                          // send the message over the network
    }

    void saveConfigurationFile(std::string path, YAML::Node& root) {
        YAML::Emitter emitter;
        emitter << root;
        std::ofstream fout(path);
        fout << emitter.c_str();
    }

    void processNullConfiguration(std::string path, YAML::Node& root, YAML::Node& yaml, ConfigurationState::Node node) { 
        std::cout << "Null " << yaml.Type() << std::endl;
    }

    void processScalarConfiguration(std::string path, YAML::Node& root, YAML::Node& yaml, ConfigurationState::Node node) { 
        if (node.has_double_value()) {
            double value = node.double_value();
            yaml = std::to_string(value);
        } else if (node.has_boolean_value()) {
            bool value = node.boolean_value();
            yaml = std::to_string(value);
        } else if (node.has_long_value()) {
            long value = node.long_value();
            yaml = std::to_string(value);
        } else if (node.has_long_value()) {
            long value = node.long_value();
            yaml = std::to_string(value);
        } else {
            yaml = node.string_value();    
        }
        saveConfigurationFile(path, root);
    }

    void processSequenceConfiguration(std::string path, YAML::Node& root, YAML::Node& yaml, ConfigurationState::Node node) { 
        std::cout << "Sequence " << yaml.Type() << std::endl;
    }

    void processMapConfiguration(std::string path, YAML::Node& root, YAML::Node& yaml, ConfigurationState::Node node) {
        // get the map from the node
        ConfigurationState::KeyPair map = node.map_value(0);
        // get the name of the current map node
        std::string name = map.name();
        // iterate through every yaml node in the map
        for (auto&& yamlNode : yaml) {
            // check if the key is the same as the name of the node
            if (yamlNode.first.as<std::string>() == name) {
                // process the value of both the map and yaml node
                processConfiguration(path, root, yamlNode.second, map.value());
                // escape any more processing
                return;
            }
        }

    }

    void processConfiguration(std::string path, YAML::Node& root, YAML::Node& yaml, ConfigurationState::Node node) {
        switch (yaml.Type()) {
            case YAML::NodeType::Undefined:
                // TODO Handle it somehow?
                break;
            case YAML::NodeType::Null:
                processNullConfiguration(path, root, yaml, node);
                break;
            // strings and numbers
            case YAML::NodeType::Scalar:
                processScalarConfiguration(path, root, yaml, node);
                break;
            // arrays and lists
            case YAML::NodeType::Sequence:
                processSequenceConfiguration(path, root, yaml, node);
                break;
            // hashes and dictionaries
            case YAML::NodeType::Map:
                processMapConfiguration(path, root, yaml, node);
                break;    
        }
    }

    void NUbugger::recvConfigurationState(const Message& message) {
        // get the root node from the message
        auto root = message.configuration_state().root();
        // get the map that contains the configuration file details
        ConfigurationState::KeyPair file = root.map_value(0);
        // get the path from the file
        std::string path = file.path();
        // load the YAML file given the path
        YAML::Node yaml = YAML::LoadFile(path);
        // process the file given the value of the file and the yaml file
        processConfiguration(path, yaml, yaml, file.value());
    }
}
}