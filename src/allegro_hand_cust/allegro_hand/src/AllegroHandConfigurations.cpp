
#include <allegro_hand/AllegroHand.h>

/**
Functions of the AllegroHand class related to configurations. 
Divided in 6 parts, groupping functions related to:
- Get and Set Configuration / state 
- Read configurations, trajectories from YAML files
- Write Configurations and trajectories to YAML files*
- Related to configurations list
- Related to trajectories
- Related to moving the hand into a configuration or trajectory
**/

/* Get and Set Configuration / state */
    //! \brief Get current state: configuration [rad], velocity [rad/s], acceleration [rad/s²], torque [Nm]
    void AllegroHand::getCurrentState(AllegroHandDriver::Vector &q_, AllegroHandDriver::Vector &vel_, AllegroHandDriver::Vector &acc_, AllegroHandDriver::Vector &torque_){
        AllegroController ->getCurrentVel(vel_);
        AllegroController ->getCurrentAcc(acc_);
        //AllegroHandDriver::getCurrentStateComm(q_);
        mutex.lock();
            q_ = q_current;
        mutex.unlock();
        mutex_torque.lock();
            torque_ = torque;
        mutex_torque.unlock();
        // std::cout << "Current state q 0 " << q_(0) << std::endl;
    }

    //! \brief Get current state: configuration [rad]
    void AllegroHand::getCurrentState(AllegroHandDriver::Vector &q_){
        mutex.lock();
            q_ = q_current;
        mutex.unlock();
    }

    //! \brief Get current state printed in deg but stored in rad: configuration [rad], velocity [rad/s], acceleration [rad/s²], torque [Nm]
    void AllegroHand::getCurrentStatePrintDeg(AllegroHandDriver::Vector &q_, AllegroHandDriver::Vector &vel, AllegroHandDriver::Vector &acc, AllegroHandDriver::Vector &torque_){
        getCurrentState(q_,vel,acc,torque_);
        std::cout << "Current hand position in degrees:" << std::endl;
        for (int i = 0; i<=3; i++){
            std::cout << "Finger " << i << ": ";
            for (int ii = 0; ii<=3; ii++){
                std::cout << "j" << i << ii << ": " << 180./M_PI*q_(i*4+ii) << ", ";
            }
            std::cout << std::endl;
        }
        std::cout << "Current hand position in rad:" << std::endl;
        for (int i = 0; i<=3; i++){
            std::cout << "Finger " << i << ": ";
            for (int ii = 0; ii<=3; ii++){
                std::cout << "j" << i << ii << ": " << q_(i*4+ii) << ", ";
            }
            std::cout << std::endl;
        }
        //std::cout << "Current hand position in degrees:" << std::endl << 180./M_PI*q_.transpose() << std::endl;
        std::cout << "Current hand velocity in degrees/s:" << std::endl << 180./M_PI*vel.transpose() << std::endl;
        std::cout << "Current hand acceleration in degrees/s^2:" << std::endl << 180./M_PI*acc.transpose() << std::endl;
        std::cout << "Current hand torque in Nm:" << std::endl << torque_.transpose() << std::endl;
    }

    //! \brief Obtain the error between the desired configuration and the current one
    //! \param q_error Error in radians
    void AllegroHand::getError(AllegroHandDriver::Vector &q_error){
        AllegroHandDriver::Vector q_;
        //AllegroHandDriver::getCurrentStateComm(q_);
        mutex.lock();
            q_ = q_current;
        mutex.unlock();
        q_error = q_desired - q_;
        std::cout << "Current hand position error in degrees:" << std::endl << 180./M_PI*(q_error).transpose() << std::endl;
    }

    //! \brief  Check and adjust the q value within the limits and set the configuration
    //! \param  q_des_ configuration desired
    //! \return boolean succesful
    bool AllegroHand::setDesiredConfiguration(Vector &q_des_) {
        bool success = true;
        // Check if q_des_ is finite and inside the limits
        for (unsigned int i = 0; i < n_fingers * finger_n_dofs; ++i) {
            if (isfinite(q_des_(i))){
                // if(q_des_(i) >= q_min[i]){
                //     if(q_des_(i) >= q_max[i]) // q_des_(i) is greater than q_max
                //     {
                //     //  std::cerr << "Configuration " << i << "changed. Bounded by q_max. q_des: " << q_des_(i) << ", q_max: " << q_max(i)<< std::endl;
                //         q_des_(i) = q_max[i]; // bound
                //     }
                // }
                // else if(q_des_(i) >= q_min[i]){
                //     //std::cerr << "Configuration " << i << "changed. Bounded by q_min. q_des: " << q_des_(i) << ", q_max: " << q_min(i)<< std::endl;
                //     q_des_(i) = q_min[i]; // q_des_(i) is bounded by q_min
                // }
                if (q_des_(i) < q_min[i]) {
                    q_des_(i) = q_min[i];
                } else if (q_des_(i) > q_max[i]) {
                    q_des_(i) = q_max[i];
                }
            }
            
            else{
                std::cerr << "Configuration " << i << " is not finite" << std::endl;
                success = false;
            }
        }
        // std::cout << "q_des_: " << q_des_ << std::endl;
        // Update the desired configuration with the configuration within the limits.
        if(success){
            mutex.lock();
                q_desired = q_des_;
            mutex.unlock();
        }
        return success;
    }

    //! \brief  Check and adjust the q value of one finger within the limits and set the configuration
    //! \param  q_finger_desired finger configuration desired
    //! \return boolean succesful
    bool AllegroHand::setDesiredConfiguration(std::vector<double> q_finger_desired, int finger_index) {
        // Check if the finger index is within bounds
        if (finger_index < 0 || finger_index >= n_fingers) {
            std::cerr << "Invalid finger index: " << finger_index << std::endl;
            return false;
        }
        // Check if the provided finger configuration has enough joint values
        if (q_finger_desired.size() != finger_n_dofs) {
            std::cerr << "Invalid size for q_finger_desired: " << q_finger_desired.size() << std::endl;
            return false;
        }
        bool success = true;

        int start_index = finger_index * finger_n_dofs;
        std::vector<double> q_finger_corrected = q_finger_desired;
        // Check if q_des_ is finite and inside the limits
        for (unsigned int i = 0; i < finger_n_dofs; ++i) {
            int index = start_index + i;
            if (std::isfinite(q_finger_corrected[i])) {
                if (q_finger_corrected[i] < q_min[index]) {
                    q_finger_corrected[i] = q_min[index];
                } else if (q_finger_corrected[i] > q_max[index]) {
                    q_finger_corrected[i] = q_max[index];
                }
            } else {
                std::cerr << "Configuration " << index << " is not finite" << std::endl;
                success = false;
            }
        }

        // Update the desired configuration with the configuration within the limits.
        if(success){
            AllegroHandDriver::Vector q_des_;
            mutex.lock();
                q_des_ = q_desired;
            mutex.unlock();

            for (int i = 0; i < finger_n_dofs; ++i) {
                q_des_(start_index + i) = q_finger_corrected[i];
            }
        
            mutex.lock();
                q_desired = q_des_;
            mutex.unlock();
        }
        return success;
    }


    //! \brief Function to assure that the q values are within the limits defined in the URDF
    //! WARNING: The URDF limits are smaller than the actual hand limits.
    bool AllegroHand::checkJointConfigurationValuesWithinLimits(std::vector<double> q_values_hand){
        bool success = true;
        for (unsigned int i=0; i < q_values_hand.size(); i++) {
            if (q_values_hand[i] < q_min[i] || q_values_hand[i] > q_max[i]) {
                std::cout << "Error in q value " << i <<": " << q_values_hand[i] << ". It is not within the limits: q_min: " << q_min[i] << " and q_max: "<< q_max[i] << std::endl;
                success = false;
            }
        }
        if(!success){
            std::cout << "Error the provided configuration values are smaller than the URDF limits" << std::endl;
        }
        return success;
    }

    //! \brief Verify that the joint positions are within the available limits and ajust them. Useful for the IK.
    //! WARNING: The URDF limits are smaller than the actual hand limits.
    bool AllegroHand::checkAndAjustJointConfigurationValues(AllegroHandDriver::Vector &q_values){
        double aux;
        bool success = true;
        bool success_one_joint = true;
        for (unsigned int i=0; i<q_values.size(); i++)
        {
            success_one_joint = true;
            // std::cout << "q_values " << i << ": "<< q_values(i) << "   ->   ";
            if(q_values(i)<=q_max[i] && q_values(i)>=q_min[i]){
                // std::cout << "Feasible(1)" << std::endl;
            }
            else{
                // std::cout << "Non feasible(-1)" ;
                success_one_joint = false;
                if(q_values(i)<0)
                {
                    if(q_values(i)<-M_PI)
                    {
                        aux = q_values(i)+2*M_PI;
                        if(aux<=q_max[i] && aux>=q_min[i])
                        {
                            q_values(i) = aux;
                            success_one_joint = true;
                            // std::cout << "   ->   " << q_values(i) << std::endl;
                        }
                    }
                }
                if(q_values(i)>0)
                {
                    if(q_values(i)>2*M_PI)
                    {
                        aux = q_values(i)-2*M_PI;
                        if(aux<=q_max[i] && aux>=q_min[i])
                        {
                            q_values(i) = aux;
                            success_one_joint = true;
                            // std::cout << "   ->   " << q_values(i) << std::endl;
                        }
                    }
                }
                success = success && success_one_joint;
            }
        }
        return success;
    }

    //! \brief Calculate the configs so that the oposite hand moves to the same position
    //! \param config_values_actual Configuration values that we want to convert
    //! \param config_values_opposite_hand Configuration values of the opposite hand
    void AllegroHand::getConfigsForTheOppositeHand(AllegroHandDriver::Vector config_values_actual, AllegroHandDriver::Vector &config_values_opposite_hand){
        config_values_opposite_hand = config_values_actual;
        // To change from one hand to the other we just need to change j00, j10, j20. This is the first joint of the fingers Index, Middle and Anular
        // Since the minimum and maximum values should be the same we only need to change the sign
        for (int i = 0; i< n_fingers-1; i++){
            config_values_opposite_hand(i*4) = -config_values_actual(i*4);
        }
    }


/* Read configurations, trajectories from YAML files*/

    //! \brief Get a list with all the filenames from the conf/folder_name folder
    //! \param folder_name Folder name located within the conf folder
    //! \param file_names_list List with all the names of the files inside the folder
    void AllegroHand::getFileNamesFromFolder(std::string folder_name, std::vector<std::string> &file_names_list)
    {
        // Check folder name.
        std::string folder_name_ = folder_name;
        // Check if folder_name has a / at first place. If so, erase it.
        if (!folder_name_.empty() && folder_name_.front() == '/') {
            folder_name_.erase(0, 1);
        }
        // Check if the folder name has / at the end. If there is no slash, then add one
        if (folder_name_.empty() || folder_name_.back() != '/') {
            folder_name_ += '/';
        }
        
        // Obtain path to the conf/folder_name/ 
        std::string folder_path = directory_conf_folder +  folder_name_;
        std::cout <<"Reading Folder: " << folder_path << std::endl;

        // Go to the conf/file_name/  directory and iterate through all the files 
        for (const auto & file_in_folder: std::filesystem::directory_iterator(folder_path)){
            file_names_list.push_back(file_in_folder.path().stem().string());
        }
    }

    /* Read Configurations */
    //! \brief Search and read all the predefined configurations from all the files in the conf/configurations.
    bool AllegroHand::searchPredefinedConfigurations(){
        // Obtain path to the conf/configurations directory
        std::string configurations_folder_path;
        if(is_left){
            configurations_folder_path = directory_conf_folder + "configurations/left/";
        }
        else{
            configurations_folder_path = directory_conf_folder + "configurations/right/";
        }
        std::cout << "Searching for the predefined configuration files available located at " << configurations_folder_path << std::endl;
        bool status_global = true;
        bool status_file;
        // Go to the conf/configurations directory and iterate through all the files 
        for (const auto & configuration_file: std::filesystem::directory_iterator(configurations_folder_path)){
            // Uploading the list of predefined configurations
            status_file = readPredefinedConfigurations(configuration_file.path().string(), configurations_list);
            status_global = status_global & status_file;
        }
        return status_global;
    }

    //! \brief Read configurations from the Yaml file and store them into the configurations_list_ map variable.
    //! \param yaml_file yaml file with a list of predefined configurations. Contains the name of the configuration and the values of the configuration.
    //! \param configurations_list_ List of predefined configurations
    bool AllegroHand::readPredefinedConfigurations(std::string yaml_file, std::map<std::string, AllegroHandDriver::Vector> &configurations_list_)
    {
        YAML::Node configuration_list_yaml;
        AllegroHandDriver::Vector config_values;
        std::cout <<"Reading File: " << yaml_file << std::endl;
        try{
            // Load existing YAML file
            configuration_list_yaml = YAML::LoadFile(yaml_file);
            // Read configurations and store them in the configurations_list_ map
            for (const auto& configuration_item_yaml : configuration_list_yaml["conf"]){
                // Read the configuration values from each configuration in the yaml file and store them in the config_values variable.
                const YAML::Node& values_node = configuration_item_yaml["values"];
                for (std::size_t i = 0; i < values_node.size(); ++i) {
                    config_values(i) = values_node[i].as<double>(); 
                }
                // Update the configurations_list_ map with the name of the configuration and its values
                configurations_list_[configuration_item_yaml["name"].as<std::string>()]=config_values;
            }
            return true;
        }catch(const YAML::ParserException& ex) {
            std::cout <<"File: " << yaml_file << std::endl;
            std::cout << ex.what() << std::endl;
            return  false;
        }
    }

    /* Read Trajectories */
    //! \brief Read a trajectory yalm, this is a configurations list and store them into the trajectories_list_ map variable.
    //! \param file_name yaml file name with a trajectory, this is a list of configurations.
    //! \param trajectories_list_ List of predefined trajectories names
    bool AllegroHand::readTrajectory(std::string file_name, std::vector<AllegroHandDriver::Vector> &trajectory_values_)
    {   
        // Check that the trajectory is within the list:
        if(std::find(trajectories_list.begin(), trajectories_list.end(), file_name) != trajectories_list.end())
        {
                // Obtain path to the conf/trajectories/left (or /right) directory
            std::string trajectory_folder_path;
            if(is_left){
                trajectory_folder_path = directory_conf_folder + name_folder_trajectories + "/left/";
            }
            else{
                trajectory_folder_path = directory_conf_folder + name_folder_trajectories + "/right/";
            }
            trajectory_folder_path += file_name + ".yaml";
            // Each trajectory is defined in a yaml file inside the trajectories folder
            // directory_conf_folder + name_folder_trajectories + "/" +name_trajectory + ".yaml"
            YAML::Node trajectory_yaml;
            AllegroHandDriver::Vector config_values;
            std::cout <<"Reading File: " << trajectory_folder_path << std::endl;

            try{
                // Load existing YAML file
                trajectory_yaml = YAML::LoadFile(trajectory_folder_path);
                // Read configurations and store them in the configurations_list_ map
                for (const auto& configuration_yaml : trajectory_yaml){
                    // Read the configuration values from each configuration in the yaml file and store them in the config_values variable.
                    const YAML::Node& values_joint = configuration_yaml;
                    for (std::size_t i = 0; i < values_joint.size(); ++i) {
                        config_values(i) = values_joint[i].as<double>(); 
                    }
                    trajectory_values_.push_back(config_values);
                }
                return true;
            }catch(const YAML::ParserException& ex) {
                std::cout <<"File: " << trajectory_folder_path << std::endl;
                std::cout << ex.what() << std::endl;
                return  false;
            }
        }
        else{
            std::cout <<"Error: File "<< file_name <<" not found in trajectories_list" << std::endl;
            return false;
        }
        
    }

    //! \brief Read trajectories from the conf/trajectories/left (or /right) folder and store them into the trajectories_list map variable.
    void AllegroHand::readTrajectories()
    {
        // Obtain path to the conf/trajectories/left (or /right) directory
        std::string trajectory_folder_path;
        if(is_left){
            trajectory_folder_path = name_folder_trajectories + "/left/";
        }
        else{
            trajectory_folder_path = name_folder_trajectories + "/right/";
        }
        getFileNamesFromFolder(trajectory_folder_path, trajectories_list);
    }

    //! \brief Read a poses trajectory yalm, this is a fingertip poses list and store the positions and orientations into two vectors.
    //! \param file_name yaml file name with a trajectory, this is a list of configurations.
    //! \param eulAngles Vector with the euler angles of the trajectory
    //! \param translations Vector with the positions (X,Y,Z cartesian) of the trajectory
    bool AllegroHand::readPosesTrajectory(const std::string file_name, std::vector<Eigen::Matrix<double, 3, 4>>& eulAngles, std::vector<Eigen::Matrix<double, 3, 4>>& translations) {
        std::string file_path = directory_conf_folder + name_folder_trajectories + "/finger_poses/" + file_name + ".yaml";

        std::cout <<"Reading File: " << file_path << std::endl;
        try{
            YAML::Node poses = YAML::LoadFile(file_path);
            for (const auto& pose : poses) {
                Eigen::Matrix<double, 3, 4> eul_angles;
                Eigen::Matrix<double, 3, 4> translation;
                
                auto euler_angles_seq = pose["euler_angles"];
                auto translation_seq = pose["coordinates"];
                
                for (int row = 0; row < 3; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        eul_angles(row, col) = euler_angles_seq[row * 4 + col].as<double>();
                        translation(row, col) = translation_seq[row * 4 + col].as<double>();
                    }
                }
                
                eulAngles.push_back(eul_angles);
                translations.push_back(translation);
            }
            return true;
        }catch(const YAML::ParserException& ex) {
            std::cout <<"File: " << file_path << std::endl;
            std::cout << ex.what() << std::endl;
            return  false;
        }
    }


/* Write Configurations and trajectories to YAML files*/

    //! \brief Write configurations onto the Yaml file. 
    //! WARNING: it reads and rewrites the whole yaml_file
    //! \param yaml_file yaml file where we will write a list of predefined configurations. With the name of the configuration and the values of the configuration.
    //! \param name_config Name of the configuration to add
    //! \param values_config Values of the configuration (16 in rad).
    bool AllegroHand::writeConfiguration(std::string yaml_file, std::string name_config, AllegroHandDriver::Vector values_config){

        YAML::Node config;
        YAML::Node new_config;
        std::vector<double> vector_;
        try{
            // Load existing YAML file
            config = YAML::LoadFile(yaml_file);

            // Info: There are two ways to solve this yaml writting. 
            // The first one (chosen) allows to write the configutation values as [1, 2, 3, ...] throush YAML::Flow but it forces to rewrite the whole YAML. We chose this option because the YAML file looks better.
            // The second one: allows to only write the new configuration added but it grows vertically (the code is below commented. To use it we should rewrite first the YAML file'configs.yaml' )

            // Read the configurations of the YAML file
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "conf";
            out << YAML::Value << YAML::BeginSeq;
            for (const auto& item : config["conf"]) {
                out << YAML::BeginMap;
                out << YAML::Key << "name";
                out << YAML::Value << item["name"].as<std::string>();
                out << YAML::Key << "values";
                out << YAML::Flow << item["values"];
                out << YAML::EndMap;
            }
            // Add the new configuration
            out << YAML::BeginMap;
            out << YAML::Key << "name";
            out << YAML::Value << name_config;
            out << YAML::Key << "values";
            out << YAML::Flow << YAML::BeginSeq;
            for (const auto& value : values_config) {
                out << value;
            }
            out << YAML::EndSeq;
            out << YAML::EndMap;

            // Rewrite the YAML file
            std::ofstream fout(yaml_file);
            fout << out.c_str();
            fout.close();

            // // Here the second option is shown:
            // new_config["name"] = name_config;
            // for (const auto& value : values_config) {
            //     vector_.push_back(value);
            // }
            // new_config["values"] = vector_;
            // config["conf"].push_back(new_config);

            // std::ofstream fout(yaml_file);
            // fout << config;
            // fout.close();

            std::cout << "New configuration has been written to " << yaml_file << std::endl;
            return true;
        }catch(const YAML::ParserException& ex) {
            std::cout <<"File: " << yaml_file << std::endl;
            std::cout << ex.what() << std::endl;
            return  false;
        }
    }

    //! \brief Write trajectories (list of configurations) onto a Yaml file. 
    //! WARNING: it creates a new yaml_file
    //! \param store_fk_trajectory True it stores the FK of the fingertips in a file at /trajectories/finger_poses
    //! \param is_left_ boolean to know if the calculated trajectory is left or right
    //! \param name_trajectory yaml filenameName where we will write a list of configurations of the trajectory.
    //! \param values_trajectory Values of the trajectory. List of configurations (each has 16 values [rad]).
    bool AllegroHand::writeTrajectories(bool store_fk_trajectory, bool is_left_, std::string name_trajectory,  std::vector<AllegroHandDriver::Vector> values_trajectory){
        try{
            // Create the data to insert in the yaml
            YAML::Emitter out;
            // out << YAML::BeginMap;
            out << YAML::BeginSeq;
            // Iterate through the list of trajectories.
            for (const auto& config : values_trajectory) {
                out << YAML::Flow;     // Use flow style for arrays
                out << YAML::BeginSeq; // Start a sequence for each configuration. This is [
                // Iterate through the values of the configuration.
                for (const auto& values_config : config) {
                    out << values_config; // Add each value to the sequence
                }
                out << YAML::EndSeq; // End the sequence for each configuration. This is ]
            }
            out << YAML::EndSeq;
            // out << YAML::EndMap;

            // Rewrite the YAML file
            std::ofstream fout;
            if(is_left_){
                fout.open(directory_conf_folder + name_folder_trajectories + "/left/" +name_trajectory + ".yaml");
                std::cout << "New trajectory file "<< name_trajectory <<"has been added to " << directory_conf_folder  << name_folder_trajectories << "/left/" << name_trajectory << ".yaml" << std::endl;
            }
            else{
                fout.open(directory_conf_folder + name_folder_trajectories + "/right/" +name_trajectory + ".yaml");
                std::cout << "New trajectory file "<< name_trajectory <<"has been added to " << directory_conf_folder  << name_folder_trajectories << "/right/"<< name_trajectory << ".yaml" << std::endl;
            
            }
            fout << out.c_str();
            fout.close();
            if(store_fk_trajectory){
                //Write the fingertip poses of the trajectory:
                std::vector<Eigen::Matrix<double, 3, 4>> eulAnglesSolved;
                std::vector<Eigen::Matrix<double, 3, 4>> translationsSolved;
                ConvertTrajectoryFromConfigurationToPoses(values_trajectory, eulAnglesSolved, translationsSolved);
                writePosesTrajectory(eulAnglesSolved, translationsSolved, name_trajectory);
                
            }
            return true;
        }catch(const YAML::ParserException& ex) {
            std::cout <<"File: " << name_folder_trajectories << std::endl;
            std::cout << ex.what() << std::endl;
            return  false;
        }
    }

    //! \brief Write the poses of a trajectory (list of coordinates and euler angles) onto a Yaml file. 
    //! WARNING: it creates a new yaml_file
    void AllegroHand::writePosesTrajectory(const std::vector<Eigen::Matrix<double, 3, 4>>& eulAngles,
                        const std::vector<Eigen::Matrix<double, 3, 4>>& translations,
                        const std::string& filename) {
        YAML::Emitter out;
        out << YAML::BeginSeq;
        for (size_t i = 0; i < eulAngles.size(); ++i) {
            out << YAML::BeginMap;
            out << YAML::Key << "pose" << YAML::Value << i;
            
            out << YAML::Key << "euler_angles" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (int col = 0; col < 4; ++col) {
                for (int row = 0; row < 3; ++row) {
                    out << eulAngles[i](row, col);
                }
            }
            out << YAML::EndSeq;
            
            out << YAML::Key << "coordinates" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (int col = 0; col < 4; ++col) {
                for (int row = 0; row < 3; ++row) {
                    out << translations[i](row, col);
                }
            }
            out << YAML::EndSeq;
            
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        std::ofstream fout(directory_conf_folder + name_folder_trajectories + "/finger_poses/" + filename + ".yaml");
        std::cout << "New trajectory file "<< filename <<"has been added to " << directory_conf_folder  << name_folder_trajectories << "/finger_poses/" << filename << ".yaml" << std::endl;

        fout << out.c_str();
        trajectories_poses_list.push_back(filename);
    }

/* Related to configurations list */
    //! \brief Define a list of predefined configurations. Only read if the yaml configurations file is empty
    void AllegroHand::setConfList(){
        // Define the configurations values
        AllegroHandDriver::Vector home, rock, paper, scissors, aloha, punk, win, botifarra, thumb_up, one;
        home << 0.0, 0.0, M_PI_4, 0.0,
            0.0, 0.0, M_PI_4, 0.0,
            0.0, 0.0, M_PI_4, 0.0,
            1.3, 0.0, M_PI_4, 0.0;
        rock <<  0.0389671, 1.75823, 1.74491, -0.134121, -0.00363929, 1.77854, 1.74606, -0.0410086, -0.0811295, 1.76435, 1.598, 0.169893, 0.241702, 0.0158886, 1.6067, 0.0833485;
        paper << -0.00914261, -0.00932013, 0.0023966, -0.00914261, 0.00683476, -0.00896508, 0.00985272, -0.00896508, 0.00896508, -0.00834374, -0.0145572, 0.229808, 0.0169538, 0.00887631, -0.00914261, 0;
        scissors << 0, -0.280314, 0.210724, -0.282799, -0.00621342, -0.218979, 0.0157998, -0.162348, -0.0820172, 1.7647, 1.598, 0.166875, 1.05504, 1.19147, 0.832067, 1.07661;
        aloha << 0.0376356, 1.75814, 1.74491, -0.134032, -0.0113617, 1.77828, 1.74606, -0.0409198, 0.573765, -0.270018, 0.264691, -0.0503287, 0.246407, -0.147879, 0.328956, 0.646639;
        punk << -0.378574, -0.0473108, 0.0442928, -0.175397, 0.0657734, 1.77606, 1.59507, -0.0848576, 0.382391, 0.032931, -0.0114504, 0.00763363, 0.242412, 0.253418, 0.343336, 0.425352;
        botifarra << -0.123203, 1.7591, 1.31609, -0.257146, 0.0462456, -0.0246762, 0.0574297, -0.0980833, 0.0355941, 1.76373, 1.22404, -0.0752712, 0.746055, 1.19085, 0.474972, 0.807567;
        win << -0.6134421927636088, -0.29345100409151936, -0.3217664518547362, 0.032576079401569154, 0.646817140597369, -0.1026102119569862, -0.5752740343367021, -0.05396800075246333, 0.4034285582564899, 1.8506231047501778, 1.8258581833522545, -0.016154894962086063, 0.3851433474752276, 1.2297248066194524, 1.5262825212806037, 1.8365097624481355;
        thumb_up << -0.2216416083534555, 1.779079998489511, 0.9282851182060222, 1.767096972006645, 0.024587395079658462, 1.9361907901537545, 0.7596351158545743, 1.8233728148965491, 0.14645921256836267, 1.8365097624481355, 1.0372862776205372, 1.7761508142381437, 0.2619400825995383, -0.10092371193347174, 0.41265992680625335, 0.09453276447594318;
        one << -0.137938, 0.0206818, 0.00781115, -0.236731, -0.0114504, 1.61291, 1.74598, -0.0583174, 0.0630219, 1.63369, 1.72653, 0.0971956, 0.29638, 1.18827, 1.12321, 1.73248;

        // Update the configurations_list map with the names and the configurations values
        configurations_list["home"] = home;
        configurations_list["rock"] = rock;
        configurations_list["paper"] = paper;
        configurations_list["scissors"] = scissors;
        configurations_list["aloha"] = aloha;
        configurations_list["punk"] = punk;
        configurations_list["botifarra"] = botifarra;
        configurations_list["win"] = win;
        configurations_list["thumb_up"] = thumb_up;
        configurations_list["one"] = one;
    }

    //! \brief Prints a list with the predefined hand configurations and its values in rad and in degrees.
    //! \param only_name True: prints only the name of the configurations. False: Prints the name and the joint values
    // \param rad_or_degrees 0: Prints the joint values in rad. 1: Prints the joint values in degrees. 2: Prints the joint values in rad and degrees. 
    void AllegroHand::getConfList(bool only_name){
        std::cout << "The available predefined configurations are: " << std::endl;
        // Print only the names of the configurations.
        if (only_name){
            std::cout << "Configuration names: " << std::endl;
            for (const auto& config : configurations_list) {
                std::cout <<  config.first << ", ";
            }
            std::cout << std::endl;
        }
        // Print the names and the configurations in rad.
        else{
            for (const auto& config : configurations_list) {
                std::cout << "Configuration name: " << config.first << std::endl;
                auto data_ = config.second;
                for (size_t i = 0; i < n_fingers * finger_n_dofs; ++i) {
                    std::cout << data_[i] << " ";
                }
                std::cout << std::endl;
            }
        }
    }

    //! \brief Get a list with the predefined hand configurations names and its values in rad.
    //! \param configurations_list_ Map of the names and the values of the defined configurations
    void AllegroHand::getConfList(std::map<std::string, AllegroHandDriver::Vector> &configurations_list_){
        configurations_list_ = configurations_list;
        // std::cout << "The available predefined configurations are: " << std::endl;
        // // Print the names and the configurations in rad.
        // for (const auto& config : configurations_list) {
        //     std::cout << "Configuration name: " << config.first << std::endl;
        //     auto data_ = config.second;
        //     for (size_t i = 0; i < 16; ++i) {
        //         std::cout << data_[i] << " ";
        //     }
        //     std::cout << std::endl;
        // }
    }

    //! \brief Allows to add a new configuration to the configuration List
    //! \param conf_name Name of the new configuration
    //! \param configuration Configuration (values in rad) 
    //! \param temporal_addition True add only to the configurations_list. False: Add to the configurations_list and write it on the yaml.
    void AllegroHand::addConf(std::string conf_name, AllegroHandDriver::Vector configuration, bool temporal_addition){
        configurations_list[conf_name] = configuration;
        AllegroHandDriver::Vector config_values_opposite_hand;
        // If it is not a temporal addition. Add it into the configurations yaml file
        if(!temporal_addition){
            writeConfiguration(config_yaml_file, conf_name, configuration);
            getConfigsForTheOppositeHand(configuration, config_values_opposite_hand);
            writeConfiguration(config_yaml_file_opposite_hand, conf_name, config_values_opposite_hand);
        }
        std::cout << "The new configuration "<< conf_name <<" has been successfully added to the configuration list" << std::endl;
    }

    //! \brief Allows to deleta a configuration from the configuration List searching it by its name
    //! \param conf_name Name of the configuration
    void AllegroHand::delConf(std::string conf_name){
        configurations_list.erase(conf_name);
        std::cout << "The configuration "<< conf_name <<" has been erased from the configuration list" << std::endl;
    }

    //! \brief Obtain the configuration values of a predefined configuration
    //! \param conf_name Name of the configuration
    //! \param configuration_values Configuration values
    void AllegroHand::getPredefinedConfigurationValues(std::string conf_name, AllegroHandDriver::Vector &configuration_values){
        // Check that the predefined configuration name exists
        auto it = configurations_list.find(conf_name);
        if (it != configurations_list.end()) {
            configuration_values = configurations_list[conf_name];
        } else {
            std::cout << "The configuration: '"<< conf_name << "' has not been found in the configurations list." << std::endl;
            std::cout << "Please enter one of the defined configurations."<< std::endl;
            getConfList(true);
        }
    }

    //! \brief Store the current hand configuration in the configuration list
    //! \param conf_name Name of the configuration
    //! \param temporal_addition True add only to the configurations_list. False: Add to the configurations_list and write it on the yaml.
    void AllegroHand::storeCurrentConf(std::string conf_name, bool temporal_addition){
        AllegroHandDriver::Vector q_;
        //AllegroHandDriver::getCurrentStateComm(q_);
        mutex.lock();
            q_ = q_current;
        mutex.unlock();
        addConf(conf_name, q_, temporal_addition);
    }


/* Related to trajectories*/
    /* Get list of trajectories*/
    //! \brief Get and print list of trajectories names.
    //! \param trajectories_list_ Map of the names and the values of the defined trajectories
    void AllegroHand::getTrajectoriesListNames(std::vector<std::string>& trajectories_list_) {
        trajectories_list_ = trajectories_list;
        std::cout << "Trajectories names: " << std::endl;
        for (const auto& trajectory : trajectories_list) {
            std::cout << trajectory << ", ";
        }
        std::cout << std::endl;
    }

    //! \brief Print list of trajectories names.
    void AllegroHand::getTrajectoriesListNames() {
        std::cout << "Trajectories names: " << std::endl;
        for (const auto& trajectory : trajectories_list) {
            std::cout << trajectory << ", ";
        }
        std::cout << std::endl;
    }

    //! \brief Get and print a list of trajectories names defined by fingertip poses.
    void AllegroHand::getTrajectoriesPosesListNames(std::vector<std::string>& trajectories_poses_list_) {
        trajectories_poses_list_ = trajectories_poses_list;
        for (const auto& pose : trajectories_poses_list) {
            std::cout << pose << ", ";
        }
        std::cout << std::endl;
    }

    //! \brief Print list of trajectories names defined by fingertip poses.
    void AllegroHand::getTrajectoriesPosesListNames() {
        for (const auto& pose : trajectories_poses_list) {
            std::cout << pose << ", ";
        }
        std::cout << std::endl;
    }

    /*Store new trajectories*/
    //! \brief Listens and reads the current hand positions and store a trajectory. Reads for a defined time but we can also stop with the stopListeningCurrentTrajectory() function
    //! \param name_trajectory yaml filenameName where we will write a list of configurations of the trajectory.
    //! \param max_time_threshold_seconds. Maximum time listening
    //! \param trajectory. The calculated trajectory
    //! \return bool true: success. false:otherwise
    bool AllegroHand::storeCurrentTrajectory(std::string name_trajectory, double max_time_threshold_seconds, std::vector<AllegroHandDriver::Vector> &trajectory){
        bool status_;
        // Store the starting time
        auto start_time = std::chrono::steady_clock::now();
        AllegroHandDriver::Vector q_current_, q_prev_;
        // Setting the Gravity Controller to allow the free movement
        setGravityController();
        listening_to_trajectory_to_store = true;
        moving = true;
        std::cout << "The currently defined trajectory is going to be stored" << std::endl;
        std::cout << " *** Start moving the hand ***" << std::endl;
        // Store the first configuration
        // Store the current configuration
        mutex.lock();
            q_current_ = q_current;
        mutex.unlock();
        trajectory.push_back(q_current_);
        q_prev_ = q_current_;
        //Start storing the configurations while  listening_to_trajectory_to_store is true
        while (listening_to_trajectory_to_store) {
            // Read the current time and calculate the elapsed time
            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = current_time - start_time;

            // Store the current configuration
            mutex.lock();
                q_current_ = q_current;
            mutex.unlock();
            if(!q_prev_.isApprox(q_current_)){
                trajectory.push_back(q_current_);
                q_prev_ = q_current_;
            }
            // If the elapsed time is bigger or equal to the input threshold time we stop the listening
            if (elapsed_seconds.count() >= max_time_threshold_seconds) {
                std::cout << "Time threshold reached. Stop listening..." << std::endl;
                break;
            }

            // Sleep for 3/4 of the control period
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000/4*3)));
        }
        listening_to_trajectory_to_store = false;
        moving = false;
        //Storing trajectory in trajectories folder
        // Calculate for the opposite hand
        std::vector<AllegroHandDriver::Vector> trajectory_opposite;
        AllegroHandDriver::Vector config_opposite;
        for (const auto& config : trajectory){
            getConfigsForTheOppositeHand(config, config_opposite);
            trajectory_opposite.push_back(config_opposite);
        }

        // Write in the conf/trajectories/ folder
        status_ = writeTrajectories(true, is_left, name_trajectory, trajectory);
        writeTrajectories(false, !is_left, name_trajectory, trajectory_opposite);
        trajectories_list.push_back(name_trajectory);
        return status_;
    }

    // //! \param name_trajectory yaml filenameName where we will write a list of configurations of the trajectory.
    // //! \param max_time_threshold_seconds. Maximum time listening
    // //! \param trajectory. The calculated trajectory
    // //! \return bool true: success. false:otherwise
    // bool AllegroHand::storeCurrentTrajectory2(std::string name_trajectory,double max_time_threshold_seconds, std::vector<AllegroHandDriver::Vector> &trajectory){
    //     // Store the starting time
    //     auto start_time = std::chrono::steady_clock::now();
    //     AllegroHandDriver::Vector q_current_, q_prev_;
    //     // Setting the Gravity Controller to allow the free movement
    //     setGravityController();
    //     listening_to_trajectory_to_store = true;
    //     std::cout << "The currently defined trajectory is going to be stored" << std::endl;
    //     std::cout << " *** Start moving the hand ***" << std::endl;

    //     // // Store the first configuration
    //     // mutex.lock();
    //     //     q_current_ = q_current;
    //     // mutex.unlock();
    //     // trajectory.push_back(q_current_);
    //     // q_prev_ = q_current_;

    //     //Start storing the configurations while  listening_to_trajectory_to_store is true
    //     while (listening_to_trajectory_to_store) {
    //         // Read the current time and calculate the elapsed time
    //         auto current_time = std::chrono::steady_clock::now();
    //         std::chrono::duration<double> elapsed_seconds = current_time - start_time;

    //         // Store the current configuration
    //         mutex.lock();
    //             q_current_ = q_current;
    //         mutex.unlock();
    //         // if(!q_prev_.isApprox(q_current_)){
    //             trajectory.push_back(q_current_);
    //         //     q_prev_ = q_current_;
    //         // }

    //         // If the elapsed time is bigger or equal to the input threshold time we stop the listening
    //         if (elapsed_seconds.count() >= max_time_threshold_seconds) {
    //             std::cout << "Time threshold reached. Stop listening..." << std::endl;
    //             break;
    //         }

    //         // Sleep for the control period since
    //         std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
    //     }
    //         //Storing trajectory in trajectories folder

    //     // // Calculate for the opposite hand
    //     // getConfigsForTheOppositeHand(q_current_, q_current_opposite);
    //     // getConfigsForTheOppositeHand(q_final, q_final_opposite);
    //     // Write in the conf/trajectories/ folder
    //     if (is_left){
    //         writeTrajectories(true, name_trajectory, trajectory);
    //         // writeTrajectories(false, name_trajectory, trajectory_opposite);
    //     }
    //     else{
    //         writeTrajectories(false, name_trajectory, trajectory);
    //         // writeTrajectories(true, name_trajectory, trajectory_opposite);
    //     }
    //     trajectories_list[name_trajectory] = trajectory;
    // }

    //! \brief Stop listening to the current trajectory. Related to storeCurrentTrajectory().
    void AllegroHand::stopListeningCurrentTrajectory(){
        listening_to_trajectory_to_store = false;
    }

    //! \brief Calculate the trajectory from an initial configuration towards a final configuration with a desired velocity
    //! \param q_initial_hand Initial configuration values(values in rad) 
    //! \param q_final_hand Final configuration values(values in rad) 
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    //! \param trajectory. The calculated trajectory
    //! \return bool true: success. false:otherwise
    bool AllegroHand::calculateTrajectory(std::vector<double> q_initial_hand, std::vector<double> q_final_hand, double percentage_velocity, std::vector<AllegroHandDriver::Vector> &trajectory){
        bool status_ = true;
        // Calculate velocity
        double percentage_velocity_ = percentage_velocity;
        double velocity_final[n_fingers * finger_n_dofs];
        // If the velocity percentage is set to 0, we set it to 0.5 rad/s
        if (percentage_velocity <= 0){
            percentage_velocity_ = 0.1; // 10% of 3rad/s = 0.3 rad/s
            std::cout << "Please enter a velocity different than 0. Setting velocity to "<< percentage_velocity_ *100 <<" %" << std::endl;
        }
        else if(percentage_velocity >= 1){
            //We set the maximum velocity to 95% the actual maximum velocity
            percentage_velocity_ = 0.95; // 95% of 3rad/s = 2.85 rad/s
            std::cout << "For safety purposes instead of the maximum velocity we set "<< percentage_velocity_ *100 <<" %" << std::endl;
        }
        for (std::size_t i = 0; i < vel_max.size(); ++i) {
            // Vmax is the same for all the qs (3rad/s). Regardless we will treat each one of the joints separately
            velocity_final[i] = vel_max[i] * percentage_velocity_;
        }
        std::cout << "percentage_velocity: " << percentage_velocity << ", velocity: " << velocity_final[0] << std::endl;

        // Check if the final configuration is within the limits
        if(checkJointConfigurationValuesWithinLimits(q_final_hand))// && checkJointConfigurationValuesWithinLimits(q_initial_hand))
        {
            // We calculate the difference between q_final - q_initial. The total radians that we would like to move
            std::cout << "q_difference (i) = q_final - q_initial" << std::endl;
            std::vector<double> q_difference(n_fingers * finger_n_dofs);
            for (std::size_t i = 0; i < q_difference.size(); ++i) {
                q_difference[i] = std::abs(q_final_hand[i] - q_initial_hand[i]); 
                std::cout << "q_difference (" << i << ") = "<< q_final_hand[i] << " - "<< q_initial_hand[i] << " = " << q_difference[i] << std::endl;
            }

            // Calculate the maximum time: There are two options. Both get the *SAME RESULT* although this depends on the fact that the maximum velocity is the same for all the joints. If this changes we must use option 2
            // OPTION 1 (FASTER): The velocity max is the same for all the joints. Thus we could do this:
            // We could also define velocity_final as a double
                // We store the maximum configuration difference
                double max_q_difference = *std::max_element(q_difference.begin(), q_difference.end());

                // Configuration difference [rad] = velocity [rad/s]* time[s]. 
                // We calculate the whole time it would take to move the joint that is the furthest away at the given velocity.      
                double max_time = max_q_difference/velocity_final[0];
                std::cout << "max_q_difference: " << max_q_difference << " max_time: " << max_time << std::endl;

            // // OPTION 2 (ROBUST): Theoretically the proper way consists in calculating the time for each joint assuming that the velocity of each joint is different (but the max velocity is the same for all the joints thus the desired velocity is the same too)
            //     std::vector<double> time_needed(n_fingers * finger_n_dofs);
            //     for (std::size_t i = 0; i < time_needed.size(); ++i) {
            //         time_needed[i] = q_difference[i]/velocity_final[i]; 
            //         // std::cout << "time needed (" << i << ") = "<< time_needed[i] << std::endl;
            //     }
            //     double max_time = *std::max_element(time_needed.begin(), time_needed.end());
            //     std::cout << " max_time_: " << max_time << std::endl;

            // We know the period at which the control loop is being executed. Thus we know how many periods we need to perform at the desired velocity. 
            // Warning: We round down the value so the velocity might be slightly higher.
            int num_motions = std::floor(max_time / control_period);
            
            std::cout << " num_motions: " << num_motions << std::endl;
            
            // We calculate the differential radiants between each motion:
            for (std::size_t i = 0; i < q_difference.size(); ++i) {
                q_difference[i] = q_difference[i]/num_motions; 
                // std::cout <<"q_final: "<< q_final(i) << " q_initial: "<< q_initial_hand[i];
                // std::cout << ", q_difference (" << i << ") = " << q_difference[i];
                // std::cout <<" q_final_teoric: "<< q_initial_hand[i] + num_motions * q_difference[i] << std::endl;
            }
            // We calculate the different configurations along the movement
            AllegroHandDriver::Vector q_motion;
            for (int index_motion = 0; index_motion <= num_motions; ++index_motion) {
                // std::cout << "Index motion: " << index_motion << std::endl;
                for (std::size_t index_joint = 0; index_joint < q_difference.size(); ++index_joint) {
                    if(q_initial_hand[index_joint]> q_final_hand[index_joint]){
                        q_motion(index_joint) = q_initial_hand[index_joint] - index_motion*q_difference[index_joint];
                    }
                    else{
                        q_motion(index_joint) = q_initial_hand[index_joint]+ index_motion*q_difference[index_joint];
                    }
                    // std::cout << "q_motion (" << index_joint << ") = q_initial "<< q_initial_hand[index_joint] << " + " << index_motion << " * q_difference "<< q_difference[index_joint] << " = " << q_motion(index_joint)  << std::endl;
                }
                trajectory.push_back(q_motion);
            }
            for (std::size_t i = 0; i < q_final_hand.size(); ++i) {
                q_motion(i) = q_final_hand[i]; 
            }
            trajectory.push_back(q_motion);
            // return true;
            std::cout << "Trajectory calculated successfully!" << std::endl;
        }
        else{
            std::cout << "Error in moveToConf with velocity parameter. Configuration out of bounds." << std::endl;
            status_ = false;
        }
        return status_;
    }

    //! \brief Calculate the trajectory from the current configuration towards a final configuration with a desired velocity
    //! \param q_final_hand Final configuration values(values in rad) 
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    //! \param trajectory. The calculated trajectory
    //! \return bool true: success. false:otherwise
    bool AllegroHand::calculateTrajectory(std::vector<double> q_final_hand, double percentage_velocity, std::vector<AllegroHandDriver::Vector> &trajectory){
        // Get current configuration
        AllegroHandDriver::Vector q_current_;
        std::vector<double> q_initial_hand(n_fingers * finger_n_dofs);
        mutex.lock();
            q_current_ = q_current;
        mutex.unlock();
        for (std::size_t i = 0; i < q_final_hand.size(); ++i) {
            q_initial_hand[i] = q_current_(i);
        }
        // Calculate trajectories 
        return calculateTrajectory(q_initial_hand, q_final_hand, percentage_velocity, trajectory);
    }

    //! \brief Calculate the trajectory from the current configuration towards a configuration that sets the fingertips into the desired poses
    //! \param desired_positions Final fingertip positions 
    //! \param desired_orientations Final fingertip orientations
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    //! \param trajectory. The calculated trajectory
    //! \return bool true: success. false:otherwise
    bool AllegroHand::calculateTrajectoryFromFingertipPoses(std::vector<double> desired_positions, std::vector<double> desired_orientations, double percentage_velocity, std::vector<AllegroHandDriver::Vector> &trajectory){
        // Get current configuration
        AllegroHandDriver::Vector q_current_;
        std::vector<double> q_initial_hand(n_fingers * finger_n_dofs);
        std::vector<double> q_final_hand(n_fingers * finger_n_dofs);
        
        mutex.lock();
            q_current_ = q_current;
        mutex.unlock();
        for (int i = 0; i < q_current_.size(); ++i) {
            q_initial_hand[i] = q_current_(i);
        }

        // Get the final configuration
        std::vector<double> q_values_hand(n_fingers * finger_n_dofs);
        AllegroHandDriver::Vector q_final_hand_, q_values_ik;
        if(computeIKHand(false, desired_positions, desired_orientations, q_values_hand)){
            for (std::size_t i = 0; i < q_values_hand.size(); ++i) {
                q_values_ik(i) = q_values_hand[i]; 
            }
            if(!is_left){
                getConfigsForTheOppositeHand(q_values_ik, q_final_hand_);
            }
            else{
                q_final_hand_ = q_values_ik;
            }
            if(checkAndAjustJointConfigurationValues(q_final_hand_)){
                for (int i = 0; i < q_final_hand_.size(); ++i) {
                    q_final_hand[i] = q_final_hand_(i); 
                }
                // Calculate trajectories 
                return calculateTrajectory(q_initial_hand, q_final_hand, percentage_velocity, trajectory);
            }
        }
        else{
            std::cout << "Error computing the Inverse Kinematics" << std::endl;
            return false;
        }
        return false;
    }

    //! \brief Calculates new trajectory from a given trajectory and a maximum velocity (recall only available with the PD or PID controller). 
    //! \param trajectory vector of configuration values of the hand.
    //! \param max_velocity_ Maximum velocity desired in rad/s
    //! WARNING: It only slows down the trajectory  
    void AllegroHand::calculateTrajectory( std::vector<AllegroHandDriver::Vector> &trajectory, double max_velocity_){
        // Check that the velocity given is within the limits
        double velocity_final;  
        // Vmax is the same for all the qs (3rad/s). 
        if(max_velocity_ > vel_max[0]){
            velocity_final = vel_max[0];
            std::cout << "Velocity max is: " << vel_max[0] << std::endl;
        }
        else if(max_velocity_ < 0){
            velocity_final = 0;
        }
        else{
            velocity_final = max_velocity_;
        }
        std::cout << "Velocity set to: " << velocity_final << std::endl;
        // Calculate the differential of joint positions to move at the desired speed
        // Velocity [rad/s] = difference_q/control_period;
        double interval_q, threshold_difference;
        interval_q = velocity_final * control_period;
        threshold_difference = 0.00005;
        std::cout << "Maximum interval set to: " << interval_q << std::endl;
        std::cout << "threshold_difference set to: " << threshold_difference << std::endl;
        
        std::vector<AllegroHandDriver::Vector> trajectory_new;
        trajectory_new.push_back(trajectory[0]);

        // 
        for (std::size_t index_motion = 0; index_motion < trajectory.size() - 1; ++index_motion) {
            // Store current and next hand configuration
            AllegroHandDriver::Vector current_pos = trajectory_new.back();
            AllegroHandDriver::Vector next_pos = trajectory[index_motion + 1];
            // Calculate the differences in each joint configuration
            AllegroHandDriver::Vector diff = next_pos - current_pos;
            // Store the maximum joint difference
            double max_joint_diff = diff.cwiseAbs().maxCoeff();
            // If the maximum joint difference is not significant we will ignore them (example if the hand is not moving)
            if (max_joint_diff < threshold_difference) {
                // trajectory_new.push_back(next_pos);
            }
            // If the maximum joint difference is bigger than the interval_q we will add configurations to adapt the velocity (make it slower) 
            else if (max_joint_diff > interval_q) {

                AllegroHandDriver::Vector step = (diff / max_joint_diff) * interval_q;
                while ((next_pos - current_pos).cwiseAbs().maxCoeff() > interval_q) {
                    current_pos += step;
                    trajectory_new.push_back(current_pos);
                }
                trajectory_new.push_back(next_pos);
            }
            // If the maximum joint difference is significant we will keep it as it is
            else {
                trajectory_new.push_back(next_pos);
            }
        }
        trajectory_new.push_back(trajectory[trajectory.size()-1]);
        trajectory = trajectory_new;
    }

    // Convert a trajectory defined with a list of Joint onfigurations to a list of Poses
    bool AllegroHand::ConvertTrajectoryFromConfigurationToPoses(std::vector<AllegroHandDriver::Vector> trajectory, std::vector<Eigen::Matrix<double, 3, 4>> &eulAnglesSolved, std::vector<Eigen::Matrix<double, 3, 4>> &translationsSolved){
        bool status_ = true;
        for (const auto& q_values_hand : trajectory) {
            Eigen::Matrix<double, 3, 4> eul_angles_solved;
            Eigen::Matrix<double, 3, 4> translation_solved;
            if (computeFK(false, std::vector<double>(q_values_hand.data(), q_values_hand.data() + q_values_hand.size()), eul_angles_solved, translation_solved)) {
                eulAnglesSolved.push_back(eul_angles_solved);
                translationsSolved.push_back(translation_solved);
            } else {
                status_ = false;
                std::cerr << "Failed to compute FK for one of the configurations." << std::endl;
            }
        }
        return status_;
    }



/* Related to moving the hand into a configuration or trajectory*/
    //! \brief Move hand to a configuration (recall only available with the PD or PID controller)
    //! \param conf Configuration (values in rad) 
    void AllegroHand::moveToConf(AllegroHandDriver::Vector conf){
        // If we are using the Torque or the GravityCompensator controller, we set up the PID.
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setPIDController();
        }
        // Moving the hand to the desired configuration
        if (!setDesiredConfiguration(conf)) {
            std::cout << "Invalid configuration, ignored" << std::endl;
        } 
    }

    //! \brief Move hand to a predefined configuration
    //! \param conf_name Name of the configuration
    void AllegroHand::moveToConf(std::string conf_name){
        // Check that the predefined configuration name exists
        auto it = configurations_list.find(conf_name);
        if (it != configurations_list.end()) {
            moveToConf(configurations_list[conf_name]);
        } else {
            std::cout << "The configuration: '"<< conf_name << "' has not been found in the configurations list." << std::endl;
            std::cout << "Please enter one of the defined configurations."<< std::endl;
            getConfList(true);
        }
    }

    //! \brief Move hand to a configuration setting the desired velocity (recall only available with the PD or PID controller)
    //! \param q_final Configuration (values in rad) 
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    void AllegroHand::moveToConf(AllegroHandDriver::Vector q_final, double percentage_velocity){
        // Verify that we have a controller able to move the hand
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setPIDController();
        }
        // TODO: check Vector or std::vector for q_values in all the cpp files (tedius)
        std::vector<double> q_final_hand(n_fingers * finger_n_dofs);
        std::vector<double> q_initial_hand(n_fingers * finger_n_dofs);

        AllegroHandDriver::Vector q_current_;
        mutex.lock();
            q_current_ = q_current;
        mutex.unlock();
        for (std::size_t i = 0; i < q_final_hand.size(); i++) {
            q_final_hand[i] = q_final(i); 
            q_initial_hand[i] = q_current_(i);
        }
        if(!moving){
            moving = true;
            std::vector<AllegroHandDriver::Vector> trajectory;
            // Calculate trajectories 
            if(calculateTrajectory(q_initial_hand, q_final_hand, percentage_velocity, trajectory)){
                for (std::size_t index_motion = 0; index_motion < trajectory.size(); index_motion++) {
                    if (!setDesiredConfiguration(trajectory[index_motion])) {
                        std::cout << "Configuration of trajectory num: "<< index_motion <<" invalid, ignored" << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
                    // usleep(control_period*1000000);
                }
            }
            else{
                std::cout << "Error in moveToConf with velocity parameter. Current configuration out of bounds.";
            }
        }
        else{
            std::cout << "The hand is already moving :(" << std::endl;
        }
        moving = false;
    }

    //! \brief Move hand to a configuration setting the desired velocity (recall only available with the PD or PID controller)
    //! \param q_final Final configuration values(values in rad) 
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    //! \param name_trajectory yaml filenameName where we will write a list of configurations of the trajectory.
    void AllegroHand::moveToConfAndSaveTrajectory(AllegroHandDriver::Vector q_final, double percentage_velocity, std::string name_trajectory){
        // Verify that we have a controller able to move the hand
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setPIDController();
        }
        // TODO: check Vector or std::vector for q_values in all the cpp files (tedius)
        std::vector<double> q_final_hand(n_fingers * finger_n_dofs);
        std::vector<double> q_initial_hand(n_fingers * finger_n_dofs);
        std::vector<double> q_final_opposite_hand(n_fingers * finger_n_dofs);
        std::vector<double> q_initial_opposite_hand(n_fingers * finger_n_dofs);

        AllegroHandDriver::Vector q_current_, q_current_opposite, q_final_opposite;

        mutex.lock();
            q_current_ = q_current;
        mutex.unlock();
        // Calculate for the opposite hand
        getConfigsForTheOppositeHand(q_current_, q_current_opposite);
        getConfigsForTheOppositeHand(q_final, q_final_opposite);

        for (std::size_t i = 0; i < q_final_hand.size(); ++i) {
            q_final_hand[i] = q_final(i); 
            q_initial_hand[i] = q_current_(i);
            q_final_opposite_hand[i] = q_final_opposite(i);
            q_initial_opposite_hand[i] = q_current_opposite(i);
        }
        if(!moving){
            moving = true;
            std::vector<AllegroHandDriver::Vector> trajectory;
            std::vector<AllegroHandDriver::Vector> trajectory_opposite;
            // Calculate trajectories 
            if(calculateTrajectory(q_initial_hand, q_final_hand, percentage_velocity, trajectory)){
                for (std::size_t index_motion = 0; index_motion < trajectory.size(); ++index_motion) {
                    if (!setDesiredConfiguration(trajectory[index_motion])) {
                        std::cout << "Configuration of trajectory num: "<< index_motion <<" invalid, ignored" << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
                    // usleep(control_period*1000000);
                }
                // Write in the conf/trajectories/ folder
                writeTrajectories(true, is_left, name_trajectory, trajectory);
                if(calculateTrajectory(q_initial_opposite_hand, q_final_opposite_hand, percentage_velocity, trajectory_opposite)){
                    writeTrajectories(false, !is_left, name_trajectory, trajectory_opposite);
                }

                trajectories_list.push_back(name_trajectory);
            }
            else{
                std::cout << "Error in moveToConf with velocity parameter. Current configuration out of bounds.";
            }
        }
        else{
            std::cout << "The hand is already moving :(" << std::endl;
        }
        moving = false;
    }

    //! \brief Move hand along a given trajectory(recall only available with the PD or PID controller)
    //! \param trajectory vector of configuration values of the hand.
    void AllegroHand::moveAlongTrajectory( std::vector<AllegroHandDriver::Vector> trajectory){
        // Verify that we have a controller able to move the hand
        if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
            std::cout << "Current controller: " << current_controller << std::endl;
            std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
            setPIDController();
        }
        if(!moving){
            moving = true;
            // Move along the configurations of the trajectory
            for (std::size_t index_motion = 0; index_motion < trajectory.size(); ++index_motion) {
                if (!setDesiredConfiguration(trajectory[index_motion])) {
                    std::cout << "Configuration of trajectory num: "<< index_motion <<" invalid, ignored" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
                // usleep(control_period*1000000);
            }
        }
        else{
            std::cout << "The hand is already moving :(" << std::endl;
        }
        moving = false;
    }

    //! \brief Move hand along a given trajectory(recall only available with the PD or PID controller)
    //! \param trajectory_name name of the trajectory, vector of configuration values of the hand.
    void AllegroHand::moveAlongTrajectory( std::string trajectory_name){
        // Check that the predefined trajectory name exists
        if(std::find(trajectories_list.begin(), trajectories_list.end(), trajectory_name) != trajectories_list.end())
        {
            std::vector<AllegroHandDriver::Vector> trajectory_values;
            readTrajectory(trajectory_name, trajectory_values);
            moveAlongTrajectory(trajectory_values);
        } else {
            std::cout << "The trajectory: '"<< trajectory_name << "' has not been found in the trajectory list." << std::endl;
            std::cout << "Please enter one of the defined trajectory_name."<< std::endl;
            getTrajectoriesListNames();
        }
    }

    //! \brief Move hand to a configuration setting the desired velocity (recall only available with the PD or PID controller)
    //! \param q_initial Configuration (values in rad) 
    //! \param q_final Configuration (values in rad) 
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    void AllegroHand::moveAlongTrajectory(AllegroHandDriver::Vector q_initial, AllegroHandDriver::Vector q_final, double percentage_velocity){
        moveToConf(q_initial, percentage_velocity);
        moveToConf(q_final, percentage_velocity);
    }

    //! \brief Move hand along a given trajectory and a maximum velocity (recall only available with the PD or PID controller). 
    //! \param trajectory vector of configuration values of the hand.
    //! \param max_velocity_ Maximum velocity desired in rad/s
    //! WARNING: It only slows down the trajectory  
    void AllegroHand::moveAlongTrajectory( std::vector<AllegroHandDriver::Vector> trajectory, double max_velocity_){
        std::vector<AllegroHandDriver::Vector> trajectory_new = trajectory;
        calculateTrajectory(trajectory_new, max_velocity_);
        moveAlongTrajectory(trajectory_new);
    }

    // // Modules the velocity of each joint separately.
        // //! \brief Move hand along a given trajectory(recall only available with the PD or PID controller). All joints move at the same velocity
        // //! \param trajectory vector of configuration values of the hand.
        // //! \param velocity_ Velocity in rad/s
        // void AllegroHand::moveAlongTrajectory( std::vector<AllegroHandDriver::Vector> trajectory, double velocity_){
        //     // Verify that we have a controller able to move the hand
        //     if ((current_controller == "Torque") || (current_controller == "GravityCompensator")){
        //         std::cout << "Current controller: " << current_controller << std::endl;
        //         std::cout << "In order to move to a desired configuration, the PD or PID controller is needed.";
        //         setPIDController();
        //     }
        //     // Check that the velocity given is within the limits
        //     double velocity_final;  
        //     // Vmax is the same for all the qs (3rad/s). 
        //     if(velocity_ > vel_max[0]){
        //         velocity_final = vel_max[0];
        //         std::cout << "Velocity max is: " << vel_max[0] << std::endl;
        //     }
        //     else if(velocity_ < 0){
        //         velocity_final = 0;
        //     }
        //     else{
        //         velocity_final = velocity_;
        //     }
        //     std::cout << "Velocity set to: " << velocity_final << std::endl;
        //     // Calculate the differential of joint positions to move at the desired speed
        //     // Velocity [rad/s] = difference_q/control_period;
        //     double interval_q, threshold_difference;
        //     interval_q = velocity_final * control_period;
        //     threshold_difference = 0.00005;
        //     std::cout << "Maximum interval set to: " << interval_q << std::endl;
        //     std::cout << "threshold_difference set to: " << threshold_difference << std::endl;
            

        //     std::vector<std::vector<double>> hand_trajectory_new;
        //     // Calculate the new trajectory acording to the new velocity
        //     // We need to take into account that joint positions are not always increasing or decreasing since we can record trajectories
        //     for (int i=0; i< n_fingers * finger_n_dofs ; i ++){
        //        std::vector<double> joint_trajectory_new;
        //        std::size_t index_motion_new = 0;
        //        joint_trajectory_new.push_back(trajectory[0](i));
        //         //    std::cout << "Joint: " << i << std::endl;
        //        for (std::size_t index_motion = 0; index_motion < trajectory.size()-1; ++index_motion) {
        //             // std::cout << "index_motion: " << index_motion;
        //             // std::cout << " joint_trajectory_new: " << joint_trajectory_new[index_motion_new];
        //             // std::cout << " trajectory: " << trajectory[index_motion+1](i) << std::endl;

        //             assert(index_motion_new < joint_trajectory_new.size());
        //             // If there is no significant difference we will keep it that way
        //             if(abs(joint_trajectory_new[index_motion_new]-trajectory[index_motion+1](i))<threshold_difference){
        //                 joint_trajectory_new.push_back(trajectory[index_motion+1](i));
        //                 index_motion_new += 1;
        //                 // std::cout << "A "<< std::endl;
        //             }
        //             // The difference is bigger than the interval_q that allows the desired velocity
        //             // We will add more points to the configuration
        //             else if(abs(joint_trajectory_new[index_motion_new]-trajectory[index_motion+1](i)) > interval_q){
        //                 // If the joint value is increasing
        //                 if(joint_trajectory_new[index_motion_new]<trajectory[index_motion+1](i)){
        //                     // Add as many configurations as needed so the difference is lower than interval_q
        //                     while(abs(joint_trajectory_new[index_motion_new]-trajectory[index_motion+1](i)) > interval_q){
        //                         joint_trajectory_new.push_back(joint_trajectory_new[index_motion_new] + interval_q);
        //                         index_motion_new += 1;
        //                     }
        //                 // std::cout << "B "<< std::endl;

        //                 }
        //                 // If the joint value is decreasing
        //                 else{
        //                     // Add as many configurations as needed so the difference is lower than interval_q
        //                     while(abs(joint_trajectory_new[index_motion_new]-trajectory[index_motion+1](i)) > interval_q){
        //                         joint_trajectory_new.push_back(joint_trajectory_new[index_motion_new] - interval_q);
        //                         index_motion_new += 1;
        //                     }
        //                 // std::cout << "C "<< std::endl;

        //                 }
        //             }
        //             // The difference is lower than the interval_q that allows the desired velocity
        //             else{
        //                 // std::cout << "D "<< std::endl;

        //                 // We check if the joint value is increasing or decreasing
        //                 // We will check for the future points to see if they keep the tendency
        //                 // The value is increasing
        //                 if(joint_trajectory_new[index_motion_new]<trajectory[index_motion+1](i)){
        //                 // std::cout << "D.1 "<< std::endl;

        //                     // While the difference is lower than interval_q we will check the next points in the trajectory and the value is increasing
        //                     while((abs(joint_trajectory_new[index_motion_new]-trajectory[index_motion+1](i)) < interval_q) && (joint_trajectory_new[index_motion_new]<trajectory[index_motion+1](i)) && (index_motion < trajectory.size()-1)){
        //                         // We keep checking points until the difference is bigger than interval_q or the tendency changes or the list finishes.
        //                         index_motion +=1;
        //                     }
        //                     // trajectory_new[index_motion_new+1](i) = trajectory[index_motion+1](i);
        //                     // index_motion_new += 1;
        //                 }
        //                 else{
        //                     // std::cout << "D.2 "<< std::endl;
        //                     // While the difference is lower than interval_q we will check the next points in the trajectory and the value is decreasing
        //                     while((abs(joint_trajectory_new[index_motion_new]-trajectory[index_motion+1](i)) < interval_q) && (joint_trajectory_new[index_motion_new]>trajectory[index_motion+1](i)) && (index_motion < trajectory.size()-1)){
        //                         // We keep checking points until the difference is bigger than interval_q or the tendency changes or the list finishes.
        //                         index_motion +=1;
        //                     }
        //                     // trajectory_new[index_motion_new+1](i) = trajectory[index_motion+1](i);
        //                     // index_motion_new += 1;
        //                 }
        //             }
        //        }
        //        hand_trajectory_new.push_back(joint_trajectory_new);
        //     }


        //     // Ensure all joint trajectories in hand_trajectory_new have the same length
        //     std::size_t max_length = 0;
        //     for (const auto& joint_trajectory : hand_trajectory_new) {
        //         if (joint_trajectory.size() > max_length) {
        //             max_length = joint_trajectory.size();
        //         }
        //     }
        //     // Add the same last value, thus we allow that joints finish at different times
        //     for (auto& joint_trajectory : hand_trajectory_new) {
        //         while (joint_trajectory.size() < max_length) {
        //             joint_trajectory.push_back(joint_trajectory.back());
        //         }
        //     }
        //     // Convert to the trajectories format
        //     std::vector<AllegroHandDriver::Vector> trajectory_new(max_length, Vector::Zero());
        //     for (std::size_t j = 0; j < max_length; ++j) {
        //         for (int i = 0; i < n_fingers * finger_n_dofs; ++i) {
        //             trajectory_new[j](i) = hand_trajectory_new[i][j];
        //         }
        //     }
            
        //     if(!moving){
        //         moving = true;
        //         // Move along the configurations of the trajectory
        //         for (std::size_t index_motion = 0; index_motion < trajectory_new.size(); ++index_motion) {
        //             if (!setDesiredConfiguration(trajectory_new[index_motion])) {
        //                 std::cout << "Configuration of trajectory num: "<< index_motion <<" invalid, ignored" << std::endl;
        //             }
        //             std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
        //             // usleep(control_period*1000000);
        //         }
        //         writeTrajectories(false, is_left, "Prova", trajectory_new);
        //     }
        //     else{
        //         std::cout << "The hand is already moving :(" << std::endl;
        //     }
        //     moving = false;
        // }
    // // End Modules the velocity of each joint separately.


    //! \brief Move hand along a given trajectory(recall only available with the PD or PID controller). All joints move at the same velocity
    //! \param trajectory vector of configuration values of the hand.
     //! \param max_velocity_ Maximum velocity desired in rad/s
    void AllegroHand::moveAlongTrajectory( std::string trajectory_name, double max_velocity_){
        // Check that the predefined trajectory name exists
        if(std::find(trajectories_list.begin(), trajectories_list.end(), trajectory_name) != trajectories_list.end())
        {
            std::vector<AllegroHandDriver::Vector> trajectory_values;
            readTrajectory(trajectory_name, trajectory_values);
            moveAlongTrajectory(trajectory_values, max_velocity_);
        } else {
            std::cout << "The trajectory: '"<< trajectory_name << "' has not been found in the trajectory list." << std::endl;
            std::cout << "Please enter one of the defined trajectory_name."<< std::endl;
            getTrajectoriesListNames();
        }
    }



    //! \brief Move hand along a trajectory defined by a list of fingertip poses
    //! \param filename name of the file that contains the poses trajectory
    void AllegroHand::moveAlongPosesTrajectory(std::string filename) {
        std::vector<Eigen::Matrix<double, 3, 4>> eulAngles;
        std::vector<Eigen::Matrix<double, 3, 4>> translations;
        readPosesTrajectory(filename, eulAngles, translations);
        
        for (size_t i = 0; i < eulAngles.size(); ++i) {
            std::vector<double> desired_position;
            std::vector<double> desired_orientation;
            
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 4; ++col) {
                    desired_orientation.push_back(eulAngles[i](row, col));
                    desired_position.push_back(translations[i](row, col));
                }
            }
            executeIK(false,desired_position, desired_orientation);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_period*1000)));
        }
    }

       //! \brief Move hand along a prismatic 3 fingers grasp
    void AllegroHand::moveGraspPrismatic3(double percentage_velocity){
        AllegroHandDriver::Vector paper, grasp3;
        paper << -0.018817789736056297, -0.018906552895188639, 0.013580763347248176, -0.0064797106166608939, 0.0060358948209991888, -0.013403237028983493, 0.017131289712541815, -0.012160552801130717, 0.013669526506380517, -0.017131289712541815, -0.0060358948209991888, 0.21107879241670693, 0.26069739837168554, 0.013935815983777538, -0.0066572369349255757, -0.0040831053200876865;
        if (is_left){
            grasp3 << -0.0035505263652936404, 1.0035562771502475, 1.4968131524486665, 0.59027500823006773, -0.10491805409442709, 1.0964913047618086, 1.2354056488039222, 0.68613922009299599, -0.079443027423445209, -0.11033260680149988, 0.049086027000184579, -0.10944497521017647, 1.3092565972020298, 0.308451977984885, -0.044026526929641145, 1.7907967354949799;
        }
        else{
            grasp3 << 0.0035505263652936404, 1.0035562771502475, 1.4968131524486665, 0.59027500823006773, 0.10491805409442709, 1.0964913047618086, 1.2354056488039222, 0.68613922009299599, 0.079443027423445209, -0.11033260680149988, 0.049086027000184579, -0.10944497521017647, 1.3092565972020298, 0.308451977984885, -0.044026526929641145, 1.7907967354949799;
        }
        moveToConf(paper, percentage_velocity);
        moveToConf(grasp3, percentage_velocity);
    }

    //! \brief Move hand along a prismatic 4 fingers grasp
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    void AllegroHand::moveGraspPrismatic4(double percentage_velocity){
        AllegroHandDriver::Vector paper, grasp4;
        paper << -0.018817789736056297, -0.018906552895188639, 0.013580763347248176, -0.0064797106166608939, 0.0060358948209991888, -0.013403237028983493, 0.017131289712541815, -0.012160552801130717, 0.013669526506380517, -0.017131289712541815, -0.0060358948209991888, 0.21107879241670693, 0.26069739837168554, 0.013935815983777538, -0.0066572369349255757, -0.0040831053200876865;
        if (is_left){
            grasp4 << -0.0037280526835583223, 0.94932198692038705, 1.4538517834286135, 0.57838074490633407, -0.018285210781262249, 1.005509066651159, 1.2676266755689618, 0.7096614572630664, 0.033996289947686609, 1.0613410937454015, 1.114332699747409, 0.9644117239728851, 1.3319799659399092, 0.33765505733942519, 0.16705226548706578, 1.5195365211865457;
        }
        else{
            grasp4 << 0.0037280526835583223, 0.94932198692038705, 1.4538517834286135, 0.57838074490633407, 0.018285210781262249, 1.005509066651159, 1.2676266755689618, 0.7096614572630664, -0.033996289947686609, 1.0613410937454015, 1.114332699747409, 0.9644117239728851, 1.3319799659399092, 0.33765505733942519, 0.16705226548706578, 1.5195365211865457;
        }
        moveToConf(paper, percentage_velocity);
        moveToConf(grasp4, percentage_velocity);
    }


    //! \brief Move hand aalong a grasp with the Pinch with the Index and Thumb finger
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    void AllegroHand::moveGraspPinchIndexThumb(double percentage_velocity){
        AllegroHandDriver::Vector paper, grasp;
        paper << -0.018817789736056297, -0.018906552895188639, 0.013580763347248176, -0.0064797106166608939, 0.0060358948209991888, -0.013403237028983493, 0.017131289712541815, -0.012160552801130717, 0.013669526506380517, -0.017131289712541815, -0.0060358948209991888, 0.21107879241670693, 0.26069739837168554, 0.013935815983777538, -0.0066572369349255757, -0.0040831053200876865;
        if (is_left){
            grasp << -0.10394165934397132, 0.4288148217683394, 1.7491668138619119, 0.42322274274300198, -0.014645921256836267, -0.019172842372585659, 0.025208737193584844, -0.02032676344130609, -0.015089737052497972, -0.027250289853628692, 0.0036392895244259818, 0.21880118726122058, 1.2080665957911612, 0.039410842654759408, 0.49822761220983014, 1.0951598573748236;
        }
        else{
            grasp << 0.10394165934397132, 0.4288148217683394, 1.7491668138619119, 0.42322274274300198, 0.014645921256836267, -0.019172842372585659, 0.025208737193584844, -0.02032676344130609, 0.015089737052497972, -0.027250289853628692, 0.0036392895244259818, 0.21880118726122058, 1.2080665957911612, 0.039410842654759408, 0.49822761220983014, 1.0951598573748236;
        }
        moveToConf(paper, percentage_velocity);
        moveToConf(grasp, percentage_velocity);
    }


    //! \brief Move hand along a grasp with the Pinch with the Middle and Thumb finger
    //! \param percentage_velocity Percentage of velocity we desire to move. (0 to 1)
    void AllegroHand::moveGraspPinchMiddleThumb(double percentage_velocity){
        AllegroHandDriver::Vector paper, grasp;
        paper << -0.018817789736056297, -0.018906552895188639, 0.013580763347248176, -0.0064797106166608939, 0.0060358948209991888, -0.013403237028983493, 0.017131289712541815, -0.012160552801130717, 0.013669526506380517, -0.017131289712541815, -0.0060358948209991888, 0.21107879241670693, 0.26069739837168554, 0.013935815983777538, -0.0066572369349255757, -0.0040831053200876865;
        if (is_left){
            grasp << -0.0081662106401753729, -0.0036392895244259818, -0.12959421233321788, -0.084768816971385663, -0.3410280573864542, 0.97151277670347236, 0.88150693334327868, 0.98393961898200011, -0.020770579236967797, -0.0048819737522787553, 0.0084325001175723953, 0.23096174006235129, 1.2544897280173757, 0.24640652975137867, 0.35860316289465766, 1.0967575942392056;
        }
        else{
            grasp << 0.0081662106401753729, -0.0036392895244259818, -0.12959421233321788, -0.084768816971385663, 0.3410280573864542, 0.97151277670347236, 0.88150693334327868, 0.98393961898200011, 0.020770579236967797, -0.0048819737522787553, 0.0084325001175723953, 0.23096174006235129, 1.2544897280173757, 0.24640652975137867, 0.35860316289465766, 1.0967575942392056;
        }
        moveToConf(paper, percentage_velocity);
        moveToConf(grasp, percentage_velocity);
    }

    //! \brief Stop the moving
    void AllegroHand::stopMovement() {
        this->moving = false;
    }

    //! \brief Get whether is moving (true) or not (false)
    bool AllegroHand::isMoving() {
        return this->moving;
    }

    //! \brief Performing coupled closing of 5º of the IMA (j1,j2,j3) and Thumb (j2, j3) 
    void AllegroHand::coupledClosing(){
        AllegroHandDriver::Vector q_coupled;
        mutex.lock();
        q_coupled = q_current;
        mutex.unlock();
        q_coupled.segment<3>(1) += 5*M_PI/180.*Eigen::Vector3d::Ones();
        q_coupled.segment<3>(5) += 5*M_PI/180.*Eigen::Vector3d::Ones();
        q_coupled.segment<3>(9) += 5*M_PI/180.*Eigen::Vector3d::Ones();
        q_coupled.segment<2>(14) += 5*M_PI/180.*Eigen::Vector2d::Ones();
        
        if (!setDesiredConfiguration(q_coupled)) {
            std::cout << "Configuration invalid, ignored" << std::endl;
        }
        std::cout << "Moving hand to configuration in degrees:" << std::endl << 180./M_PI*q_desired.transpose() << std::endl;
        moveToConf(q_desired);
    }

    //! \brief Performing coupled closing of the IMA (j1,j2,j3) and Thumb (j2, j3)
    //! \param coupling_value Value to couple in degrees. 
    void AllegroHand::coupledClosing(double coupling_value){
        AllegroHandDriver::Vector q_coupled;
        mutex.lock();
        q_coupled = q_current;
        mutex.unlock();
        q_coupled.segment<3>(1) += coupling_value*M_PI/180.*Eigen::Vector3d::Ones();
        q_coupled.segment<3>(5) += coupling_value*M_PI/180.*Eigen::Vector3d::Ones();
        q_coupled.segment<3>(9) += coupling_value*M_PI/180.*Eigen::Vector3d::Ones();
        q_coupled.segment<2>(14) += coupling_value*M_PI/180.*Eigen::Vector2d::Ones();
        
        if (!setDesiredConfiguration(q_coupled)) {
            std::cout << "Configuration invalid, ignored" << std::endl;
        }
        std::cout << "Moving hand to configuration in degrees:" << std::endl << 180./M_PI*q_desired.transpose() << std::endl;
        moveToConf(q_desired);
    }

    //! \brief Performing coupled closing of all the joints at the desired couple values
    //! \param coupling_value Value to couple in degrees. 16 values, one per joint.
    void AllegroHand::coupledClosing(std::array<double, n_fingers*finger_n_dofs> coupling_value){
        AllegroHandDriver::Vector q_coupled;
        mutex.lock();
            q_coupled = q_current;
        mutex.unlock();

        if(coupling_value.size() != n_fingers * finger_n_dofs){
            std::cout << "Dimension mismatch of the coupling_value (coupledClosing)" << std::endl;
        }
        else{
            for(size_t i = 0; i < n_fingers*finger_n_dofs; ++i){
                q_coupled(i) += coupling_value[i]*M_PI/180.;
            }
            if (!setDesiredConfiguration(q_coupled)) {
                std::cout << "Configuration invalid, ignored" << std::endl;
            }
            std::cout << "Moving hand to configuration in degrees:" << std::endl << 180./M_PI*q_desired.transpose() << std::endl;
            moveToConf(q_desired);
        }
    }

    //! \brief Performing coupled closing of each finger the desired value. For the IMA (j1,j2,j3) and Thumb (j2, j3)
    //! \param coupling_value Value to couple in degrees. One value per finger
    void AllegroHand::coupledClosing(std::array<double, n_fingers> coupling_value){
        AllegroHandDriver::Vector q_coupled;
        mutex.lock();
            q_coupled = q_current;
        mutex.unlock();

        if(coupling_value.size() != n_fingers){
            std::cout << "Dimension mismatch of the coupling_value (coupledClosing)" << std::endl;
        }
        else{
            q_coupled.segment<3>(1) += coupling_value[0]*M_PI/180.*Eigen::Vector3d::Ones();
            q_coupled.segment<3>(5) += coupling_value[1]*M_PI/180.*Eigen::Vector3d::Ones();
            q_coupled.segment<3>(9) += coupling_value[2]*M_PI/180.*Eigen::Vector3d::Ones();
            q_coupled.segment<2>(14) += coupling_value[3]*M_PI/180.*Eigen::Vector2d::Ones();
            
            if (!setDesiredConfiguration(q_coupled)) {
                std::cout << "Configuration invalid, ignored" << std::endl;
            }
            std::cout << "Moving hand to configuration in degrees:" << std::endl << 180./M_PI*q_desired.transpose() << std::endl;
            moveToConf(q_desired);
        }
    }
