
#include <iostream>
#include <opencv2/highgui.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <gflags/gflags.h>
#include <glm/gtc/matrix_inverse.hpp>
//#include <kindr/minimal/quat-transformation.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


#include <vrglasses_for_robots/csv_processor.h>

DEFINE_string(pose_file, "/media/lucas/ntfs1t/irchel-sandro/base-op/image_poses.txt", "visensor simulator project folder");
// /media/secssd/dataset/amazon_models/irchel140821/random_poses.txt
DEFINE_string(output_folder_path, "/media/secssd/tmp/render_test/a1", "result path");
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

//DEFINE_string(mesh_obj_file, "/media/secssd/dataset/amazon_models/irchel140821/3dmesh/irchel140821-merged_simplified_3d_mesh.obj", "compiled shader folders");
//DEFINE_string(mesh_texture_file, "/media/secssd/dataset/amazon_models/irchel140821/3dmesh/irchel140821-merged_semantic-rgbs.png", "compiled shader folders");
//DEFINE_string(model_folder, "", "compiled shader folders");
//DEFINE_string(model_list_file, "", "compiled shader folders");
//DEFINE_string(model_pose_file, "", "compiled shader folders");

// /media/secssd/sa_dataset/v4rl_aerial_semantic_dataset/scenes/buffalo_construction_site_v1.scn
// /media/secssd/sa_dataset/v4rl_aerial_semantic_models

//DEFINE_string(mesh_obj_file, "/media/secssd/code/vrglasses4robots/data/models/50s_house_v2_45_3_Zu_Xf.obj", "compiled shader folders");
//DEFINE_string(mesh_texture_file, "/media/secssd/code/vrglasses4robots/data/textures/new_texture_small.tga", "compiled shader folders");

DEFINE_string(mesh_obj_file, "", "compiled shader folders");
DEFINE_string(mesh_texture_file, "", "compiled shader folders");
DEFINE_string(model_folder, "/media/lucas/ntfs1t/irchel-sandro/base-op/", "compiled shader folders");
DEFINE_string(model_list_file, "/media/lucas/ntfs1t/irchel-sandro/base-op/irchel1408/model_def_list.txt", "compiled shader folders");
DEFINE_string(model_pose_file, "/media/lucas/ntfs1t/irchel-sandro/base-op/irchel1408/model_poses_list.txt", "compiled shader folders");






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
#ifndef _DEBUG
         //return false;
#endif
    }

    //visim_data_source_ = DataSource::Create(visim_project_folder, camera_id);


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

    }
    else
    {
        render_app->buildPerpectiveProjection(projection_matrix_,FLAGS_output_w,FLAGS_output_h,FLAGS_fx,FLAGS_fy,0,FLAGS_cx,FLAGS_cy,FLAGS_near,FLAGS_far);
    }

}

void CSVProcessor::parseLine(std::string line,std::vector<std::string>& vec)
{
    //std::vector<std::string> vec;
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
    //glm::mat4 result(1.0);
    glm::mat3 rot = glm::mat3(in_mat);
    glm::mat3 rott = glm::transpose(rot);
    glm::mat4 result(rott);

    glm::vec3 t = in_mat[3];
    glm::vec3 t_out = -(rott * t);
    result[3] = glm::vec4(t_out,1.0);
    return result;
}

//void CSVProcessor::kindr2mvp(glm::vec3 position, glm::quat orientation,glm::mat4& mvp){
//    Eigen::Vector3d p_WS(
//        position.x,
//        position.y,
//        position.z);

//    Eigen::Quaterniond q_WS;
//    q_WS.x() = orientation.x;
//    q_WS.y() = orientation.y;
//    q_WS.z() = orientation.z;
//    q_WS.w() = orientation.w;

//    kindr::minimal::QuatTransformation T_WC =
//        kindr::minimal::QuatTransformation(p_WS, q_WS);
//    kindr::minimal::QuatTransformation T_CW_cv = T_WC.inverse();
//    auto T_CW_cv_eigen = T_CW_cv.getTransformationMatrix();
//    glm::mat4 T_CW_cv_glm;
//    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1);

//    for (int i = 0; i < 4; ++i)
//    {
//        for (int j = 0; j < 4; ++j)
//        {
//            T_CW_cv_glm[j][i] = T_CW_cv_eigen(i, j);
//        }
//    }
//    std::cout << " twc " << T_WC.getTransformationMatrix() << std::endl;
//    std::cout << " tcw " << glm::to_string(T_CW_cv_glm) << std::endl;
//    std::cout << " conversion_gl_cv " << glm::to_string(T_CW_cv_glm) << std::endl;
//    mvp = projection_matrix_ * conversion_gl_cv * T_CW_cv_glm;
//}
void CSVProcessor::glm2mvp(glm::vec3 position, glm::quat orientation,glm::mat4& mvp){
    glm::mat4 T_WC = glm::mat4_cast(orientation);
    //std::cout << glm::to_string(T_WC) << std::endl;
    T_WC[3] = glm::vec4(position,1.0);
    //std::cout << glm::to_string(T_WC) << std::endl;

    glm::mat4 T_CW = pose_inverse(T_WC);
    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,
                                           0,-1,0,0,
                                           0,0,-1,0,
                                           0,0,0,1);
    mvp = projection_matrix_ * conversion_gl_cv * T_CW ;
    //mvp = projection_matrix_ * glm::lookAt(glm::vec3(0.1,0,50),glm::vec3(0,0,0),glm::vec3(0,0,1));

    //std::cout << "lookat " << glm::to_string(glm::lookAt(glm::vec3(5,0,5),glm::vec3(0,0,0),glm::vec3(0,0,1))) << std::endl;
    //std::cout << "rot " << glm::to_string(rot) << std::endl;
    //std::cout << "translation' " << glm::to_string(translation) << std::endl;
}

void CSVProcessor::renderPose(glm::vec3 position, glm::quat orientation)
{
    static long int counter = 0;
    counter++;
    std::cout << '\r' << counter << std::flush;
    //std::cout << glm::to_string(position) << " " << glm::to_string(orientation) <<std::endl;


//    glm::mat4 lookat = glm::lookAt(glm::vec3(5,0,5),glm::vec3(0,0,0),glm::vec3(0,0,1));
//    glm::mat4 lookat_inv = pose_inverse(lookat);
//    glm::quat rot = glm::quat_cast(lookat_inv);
//    glm::vec4 translation = lookat_inv[3];

    glm::mat4 mvp;
    //kindr2mvp(position,orientation,mvp);
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

//    std::cout << "mvp " << glm::to_string(conversion_gl_cv * T_CW) << std::endl;
//    std::cout << "translate " <<  glm::to_string(projection_matrix_)<< std::endl;

//    glm::mat4 T_WC = glm::translate(rotation_mat,position);
//    glm::affineInverse(T_WC);
}

//glm::mat4 CSVProcessor::computeMVP(DataEntry &entry)
//{
//    kindr::minimal::QuatTransformation T_CW_cv = entry.T_WC.inverse();
//    auto T_CW_cv_eigen = T_CW_cv.getTransformationMatrix();
//    glm::mat4 T_CW_cv_glm;
//    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1);

//    for (int i = 0; i < 4; ++i)
//    {
//        for (int j = 0; j < 4; ++j)
//        {
//            T_CW_cv_glm[j][i] = T_CW_cv_eigen(i, j);
//        }
//    }
//    return perpective_ * conversion_gl_cv * T_CW_cv_glm;
//}

//{
//    //VisimProject project =  visim_data_source_->getProject();
//    double near = 0.1, far = 500.0;
//    vrglasses_for_robots::VulkanRenderer app = vrglasses_for_robots::VulkanRenderer(project.w,project.h,near,far,"/media/secssd/catkin_ws/src/vulkan_glasses_for_robots/vrglasses_for_robots/shaders");
//    cv::Mat result_depth_map, result_rgb_map, result_semantic_map;

//    app.loadMesh("/media/secssd/code/vrglasses4robots/data/models/50s_house_v2_45_3_Zu_Xf.obj","/media/secssd/code/vrglasses4robots/data/textures/new_texture_small.tga");

//    try {
//        size_t count = 0;
//        glm::mat4 empty;

//        buildOpenglProjectionFromIntrinsics(perpective_,empty,project.w,project.h,project.f,project.f,0,project.cx,project.cy,near,far);
//        while(count < 1000){
//            DataEntry current = visim_data_source_->at(count);
//            std::cout << current.sequence << std::endl;
//            glm::mat4 mvp = computeMVP(current);
//            app.setCamera(mvp);
//            cv::Mat show_img, channels[4];
//            app.renderMesh(result_depth_map, result_semantic_map);
//            cv::split(result_semantic_map,channels);
//            cv::imshow("RGB",result_semantic_map);
//            //cv::imshow("Semantics",channels[3]);
//            if (1)
//            {
//                    double min_depth = 0, max_depth = 30;
//                    //cv::minMaxLoc(mesh_depth_image, &min_depth, &max_depth);

//                    result_depth_map.convertTo(
//                        show_img, CV_8U, 255.0 / (max_depth - min_depth),
//                        -min_depth * 255.0 / (max_depth - min_depth));
//                    //cv::imshow("depth map render", show_img);
//                    //LOG(INFO) << "rende " << min_depth << " / " << max_depth << " - "
//                    //          << okvis_reader.getNextId() << " / " << okvis_reader.size();
//            }
//            count = (count +1) % visim_data_source_->size();
//            cv::waitKey(1);
//        }
//    } catch (const std::exception& e) {
//        std::cerr << e.what() << std::endl;
//        return;
//    }
//}



//glm::mat4 CSVProcessor::computeMVP(DataEntry &entry)
//{
//    kindr::minimal::QuatTransformation T_CW_cv = entry.T_WC.inverse();
//    auto T_CW_cv_eigen = T_CW_cv.getTransformationMatrix();
//    glm::mat4 T_CW_cv_glm;
//    glm::mat4 conversion_gl_cv = glm::mat4(1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1);

//    for (int i = 0; i < 4; ++i)
//    {
//        for (int j = 0; j < 4; ++j)
//        {
//            T_CW_cv_glm[j][i] = T_CW_cv_eigen(i, j);
//        }
//    }
//    return perpective_ * conversion_gl_cv * T_CW_cv_glm;
//}

//void CSVProcessor::buildOpenglProjectionFromIntrinsics(glm::mat4 &matPerspective, glm::mat4 &matProjection/*, glm::mat4 &matCVProjection*/, int img_width, int img_height, float alpha, float beta, float skew, float u0, float v0, float near, float far) {
//    // These parameters define the final viewport that is rendered into by the
//    // camera.
//    float l = 0;
//    float r = img_width;
//    float b = 0;
//    float t = img_height;

//    // near and far clipping planes, these only matter for the mapping from
//    // world-space z-coordinate into the depth coordinate for OpenGL
//    float n = near;
//    float f = far;

//    //  // set the viewport parameters
//    //  viewport[0] = l;
//    //  viewport[1] = b;
//    //  viewport[2] = r - l;
//    //  viewport[3] = t - b;

//    // construct an orthographic matrix which maps from projected coordinates to
//    // normalized device coordinates in the range
//    // [-1, 1].  OpenGL then maps coordinates in NDC to the current viewport
//    glm::mat4 ndc(0);
//    ndc[0][0] = 2.0 / (r - l);
//    ndc[3][0] = -(r + l) / (r - l);
//    ndc[1][1] = 2.0 / (t - b);
//    ndc[3][1] = -(t + b) / (t - b);
//    ndc[2][2] = -2.0 / (f - n);
//    ndc[3][2] = -(f + n) / (f - n);
//    ndc[3][3] = 1.0;

//    // construct a projection matrix, this is identical to the projection matrix
//    // computed for the intrinsic,
//    // except an additional row is inserted to map the z-coordinate to OpenGL.
//    // CMatrix4<T> matProjection(0);    // the 3rd column is inverted to make the
//    // camera look towards +Z (instead of -Z in opengl)
//    matProjection = glm::mat4(0);
//    matProjection[0][0] = alpha;
//    matProjection[1][0] = skew;
//    matProjection[2][0] = -u0;
//    matProjection[1][1] = beta;
//    matProjection[2][1] = -v0;
//    matProjection[2][2] = (n + f);
//    matProjection[3][2] = n * f;
//    matProjection[2][3] = -1.0;

////    matCVProjection = glm::mat4(0);
////    matCVProjection[0][0] = alpha;
////    matCVProjection[1][0] = skew;
////    matCVProjection[2][0] = u0;
////    matCVProjection[1][1] = beta;
////    matCVProjection[2][1] = v0;
////    matCVProjection[2][2] = 1;
////    matCVProjection[3][2] = 0;
////    matCVProjection[2][3] = 1;
////    matCVProjection[3][3] = 1;

//    // resulting OpenGL frustum is the product of the orthographic
//    // mapping to normalized device coordinates and the augmented camera intrinsic
//    // matrix
//    matPerspective = ndc * matProjection;
//    matPerspective[1][1] *= -1; //was originally designed for OpenGL, where the Y coordinate of the clip coordinates is inverted in relation Vulkan.
//}
