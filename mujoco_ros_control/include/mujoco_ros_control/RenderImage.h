//
// Created by aleksi on 29.10.2018.
//

#ifndef RENDER_IMAGE_H
#define RENDER_IMAGE_H

#include <mujoco.h>
#include <opencv2/highgui/highgui.hpp>
#include <glfw3.h>
#include <sensor_msgs/Image.h>

class RenderImage {
public:

    explicit RenderImage(const mjModel *m);
    explicit RenderImage(const mjModel* m, const float camera_look_at[3]);

    ~RenderImage();

    /** @brief Update the scene info stored in internal variables */
    void updateScene(const mjModel* m, mjData* d);

    /** @brief Render tuple of RGB and Depth image from the current scene.
     * Be sure to call updateScene before rendering. */
    std::tuple<cv::Mat, cv::Mat> render();


    /** @brief Create OpenGL context/window which is invisible and used for rendering. */
    static void initOpenGL();

    /** @brief Close OpenGL window */
    static void closeOpenGL();

private:
    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjrContext con;
    mjrRect viewport;
};


#endif //RENDER_IMAGE_H