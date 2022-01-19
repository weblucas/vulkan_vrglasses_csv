
#include <iostream>
#include <opencv2/highgui.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <gflags/gflags.h>
#include <glm/gtc/matrix_inverse.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


#include <vrglasses_for_robots/csv_processor.h>


DEFINE_string(output_folder_path, "/media/secssd/tmp/render_test/a44", "result path");
DEFINE_int32(step_skip, 1, "step skip");
DEFINE_string(resource_folder, "/media/secssd/tmp/render_test/a13", "result path");

DEFINE_int32(output_w, 752, "width");
DEFINE_int32(output_h, 480, "height");
DEFINE_double(far, 1000, "far");
DEFINE_double(near, 0.1, "near");
DEFINE_double(fx, 571.63, "fx");
DEFINE_double(fy, 571.63, "fy");
DEFINE_double(cx, 366.23, "cx");
DEFINE_double(cy, 243.592, "cy");
DEFINE_bool(ortho,false,"use orthographic projection");
DEFINE_string(shader_folder, "/media/secssd/code/vrglasses4robots/data/shaders", "compiled shader folders");

DEFINE_string(mesh_obj_file, "", "compiled shader folders");
DEFINE_string(mesh_texture_file, "", "compiled shader folders");
DEFINE_string(model_folder, "/media/lucas/ntfs1t/irchel-sandro/base-op/", "compiled shader folders");
DEFINE_string(model_list_file, "/media/lucas/ntfs1t/irchel-sandro/base-op/irchel1408/model_def_list.txt", "compiled shader folders");
DEFINE_string(model_pose_file, "/media/lucas/ntfs1t/irchel-sandro/base-op/irchel1408/model_poses_list.txt", "compiled shader folders");
DEFINE_string(pose_file, "/media/lucas/ntfs1t/irchel-sandro/base-op/image_poses.txt", "visensor simulator project folder");





bool CSVProcessor::initialization(
    ) { //const std::string& csv_pose_file, const std::string& output_folder_path
    if (!boost::filesystem::exists(FLAGS_output_folder_path)) {
        if (!boost::filesystem::create_directories(FLAGS_output_folder_path)) {
            LOG(ERROR) << "the output folder could not be created :"
                       << FLAGS_output_folder_path.c_str();
            return false;
        }
    } else {
        LOG(ERROR)
                << "the output folder already exist - please delete it or "
                   "change the arg:"
                << FLAGS_output_folder_path.c_str();

         return false;
    }


    pose_file_.open (FLAGS_pose_file, std::ifstream::in);

    if (!pose_file_.is_open()) {
        LOG(ERROR) << "could not open the pose file";
        return false;
    }

    output_folder_ = FLAGS_output_folder_path;

    initVulkan();

    return true;
}

void CSVProcessor::initVulkan()
{
    render_app = new vrglasses_for_robots::VulkanRenderer(FLAGS_output_w,FLAGS_output_h,FLAGS_near,FLAGS_far,FLAGS_shader_folder);

#if 1
    if(!FLAGS_model_folder.empty() && !FLAGS_model_list_file.empty())
    {
        render_app->loadMeshs(FLAGS_model_folder,FLAGS_model_list_file);

        if(!FLAGS_model_pose_file.empty())
        {
            LOG(INFO) << "Load with pose file: " << FLAGS_model_pose_file;
            render_app->loadScene(FLAGS_model_pose_file);
        }
        else
        {
            LOG(INFO) << "Load without scene file";
            render_app->noFileScene();
        }

    }
    else if( !FLAGS_mesh_obj_file.empty() &&  !FLAGS_mesh_texture_file.empty())
    {
        // Load Mesh
        LOG(INFO) << "Loading single file " << FLAGS_mesh_obj_file;
        render_app->loadMesh(FLAGS_mesh_obj_file,FLAGS_mesh_texture_file);
        render_app->noFileScene();
    }
    else{
        LOG(ERROR) << "mesh_obj_file and mesh_texture_file need to be defined parameter, alternatively model_folder and model_list_file";
    }
#endif
    if(FLAGS_ortho)
    {
        //todo
    }
    else
    {
        render_app->buildPerpectiveProjection(projection_matrix_,FLAGS_output_w,FLAGS_output_h,FLAGS_fx,FLAGS_fy,0,FLAGS_cx,FLAGS_cy,FLAGS_near,FLAGS_far);
    }

}

void CSVProcessor::parseLine(std::string line,std::vector<std::string>& vec)
{
    boost::tokenizer<boost::escaped_list_separator<char> > tk(
        line, boost::escaped_list_separator<char>('\\', ',', '\"'));
    for (boost::tokenizer<boost::escaped_list_separator<char> >::iterator i(tk.begin());
         i!=tk.end();++i)
    {
        vec.push_back(*i);
    }
}




void CSVProcessor::runHeadless()
{
    std::string line;
    std::vector<std::string> vec;
    std::getline(pose_file_,line);

    while (!pose_file_.eof())
    {
        vec.clear();
        std::getline(pose_file_,line);
        parseLine(line,vec);
        if(vec.size() != 8)
            continue;
        try
        {
            glm::vec3 position;
            glm::quat orientation;

            std::string id = vec[0] ;
            position.x = boost::lexical_cast<double>(vec[1]);
            position.y = boost::lexical_cast<double>(vec[2]);
            position.z = boost::lexical_cast<double>(vec[3]);

            orientation.x = boost::lexical_cast<double>(vec[4]);
            orientation.y = boost::lexical_cast<double>(vec[5]);
            orientation.z = boost::lexical_cast<double>(vec[6]);
            orientation.w = boost::lexical_cast<double>(vec[7]);
            orientation = glm::normalize(orientation);
            renderPose(position,orientation);
        }
        catch(boost::bad_lexical_cast &)
        {
            LOG(INFO) << "#"<<  "cast issue" << "#";
            continue;
        }
    }

    LOG(INFO) << "#"<<  "stopVulkan" << "#";
    stopVulkan();
}



void CSVProcessor::stopVulkan()
{
    delete render_app;
    render_app = nullptr;
}

/*!
 * @brief inverse orthonormal rotation + translation matrix (ridig-body)
 *
 * @code
 * X = | R  T |   X' = | R' -R'T |
 *     | 0  1 |        | 0     1 |
 * @endcode
 *
 */

glm::mat4 pose_inverse(glm::mat4 in_mat)
{
    glm::mat3 rot = glm::mat3(in_mat);
    glm::mat3 rott = glm::transpose(rot);
    glm::mat4 result(rott);

    glm::vec3 t = in_mat[3];
    glm::vec3 t_out = -(rott * t);
    result[3] = glm::vec4(t_out,1.0);
    return result;
}

void CSVProcessor::glm2mvp(glm::vec3 position, glm::quat orientation,glm::mat4& mvp){
    glm::mat4 T_WC = glm::mat4_cast(orientation);

    T_WC[3] = glm::vec4(position,1.0);

    glm::mat4 T_CW = pose_inverse(T_WC);
    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,
                                           0,-1,0,0,
                                           0,0,-1,0,
                                           0,0,0,1);
    mvp = projection_matrix_ * conversion_gl_cv * T_CW ;

}

void CSVProcessor::renderPose(glm::vec3 position, glm::quat orientation)
{
    static long int counter = 0;
    counter++;
    std::cout << '\r' << counter << std::flush;

    glm::mat4 mvp;

    glm2mvp(position,orientation,mvp);
    std::cout << " mvp " << glm::to_string(mvp) << std::endl;
    std::cout << " perpective " << glm::to_string(projection_matrix_) << std::endl;
    render_app->setCamera(mvp);
    cv::Mat show_img, channels[4];
    cv::Mat result_depth_map, result_rgb_map, result_semantic_map;
    render_app->renderMesh(result_depth_map, result_semantic_map);
    cv::split(result_semantic_map,channels);
    cv::imshow("RGB",result_semantic_map);
    cv::imshow("Semantics",channels[3]);
    if (1)
    {
        double min_depth = 0, max_depth = 100;
        //cv::minMaxLoc(mesh_depth_image, &min_depth, &max_depth);

        result_depth_map.convertTo(
            show_img, CV_8U, 255.0 / (max_depth - min_depth),
            -min_depth * 255.0 / (max_depth - min_depth));
        cv::imshow("depth map render", show_img);
        //LOG(INFO) << "rende " << min_depth << " / " << max_depth << " - "
        //          << okvis_reader.getNextId() << " / " << okvis_reader.size();
    }

    cv::waitKey(0);

}

