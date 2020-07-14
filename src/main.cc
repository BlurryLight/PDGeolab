

// GLAD
#include "glsupport.h"
#include "resource_path_searcher.h"
#include <glad/glad.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <fstream>
#include <iostream>
#include <sstream>
using namespace PD;
static const GLuint SCR_WIDTH = 800;
static const GLuint SCR_HEIGHT = 800;

static float deltaTime = 0.0f;
static float lastFrame = 0.0f;
void processInput(GLFWwindow *window, Camera &cam);
void framebuffer_size_callback(GLFWwindow *, int width, int height);
void mouse_callback(GLFWwindow *, double width, double height);
void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mods);
static bool firstMouse = true;
static double lastX, lastY;
static Camera cam;
static bool AllowMouseMove = true;
int main() {
  if (!glfwInit()) {
    std::cerr << "FATAL INIT FAILED" << std::endl;
    return -1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow *window =
      glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "PD_GEO_LAB", NULL, NULL);

  if (window == NULL) {
    std::cerr << "GLFW INIT FAILED" << std::endl;
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetKeyCallback(window, key_callback);

  // glad loads
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "FAILED TO INIT GLAD" << std::endl;
    return -1;
  }

  // config global OpenGL state
  glEnable(GL_DEPTH_TEST);
  unsigned int texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, SCR_WIDTH, SCR_HEIGHT, 0,
               GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                         texture, 0);
  glBindTexture(GL_TEXTURE_2D, 0);

  const char *glsl_verson = "#version 330";
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_verson);

  // IMGUI
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  float Blinn_Shininess = 8.0f;

  // shader
  ResourcePathSearcher resourcesPath;
  resourcesPath.add_path(
      (ResourcePathSearcher::root_path / "resources" / "models").u8string());
  resourcesPath.add_path(
      (ResourcePathSearcher::root_path / "resources" / "shaders").u8string());

  Shader shader(resourcesPath.find_path("simple.vert"),
                resourcesPath.find_path("simple.frag"));
  Model model(resourcesPath.find_path("cube.obj"));
  cam = Camera();
  // main loop
  while (!glfwWindowShouldClose(window)) {

    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
    processInput(window, cam);

    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    {
      ImGui::Begin("Background Color"); // Create a window called "Hello,
                                        // world!" and append into it.
      ImGui::ColorEdit3(
          "clear color",
          (float *)&clear_color); // Edit 3 floats representing a color

      if (ImGui::CollapsingHeader("Camera")) {
        ImGui::SliderFloat("Blinn Shininess", &Blinn_Shininess, 0.0f, 128.0f);

        ImGui::SliderFloat("Camera Movementspeed", &cam.MovementSpeed, 0.0f,
                           100.0f);
        ImGui::SliderFloat("Camera MouseSensitivity", &cam.MouseSensitivity,
                           0.0f, 1.0f);
        ImGui::Text("Cam: (%.3f,%.3f,%.3f)", cam.Position.x, cam.Position.y,
                    cam.Position.z);
      }

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();
    }
    ImGui::Render();

    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto projection = glm::perspective(
        glm::radians(cam.Zoom), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 2000.0f);
    auto view = cam.GetViewMatrix();
    shader.use();
    shader.setMat4("model", glm::mat4(1.0));
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    model.Draw(shader);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwTerminate();
  return 0;
}
void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  // make sure the viewport matches the new window dimensions; note that width
  // and height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window, Camera &cam) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    cam.ProcessKeyboard(FORWARD, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    cam.ProcessKeyboard(BACKWARD, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    cam.ProcessKeyboard(LEFT, deltaTime);
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    cam.ProcessKeyboard(RIGHT, deltaTime);
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos) {
  if (AllowMouseMove) {
    if (firstMouse) {
      lastX = xpos;
      lastY = ypos;
      firstMouse = false;
    }
    float xoffset = xpos - lastX;
    float yoffset =
        lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    cam.ProcessMouseMovement(xoffset, yoffset);
  } else {
    firstMouse = true;
  }
};

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mods) {

  if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
    AllowMouseMove = !AllowMouseMove;
    std::cout << (AllowMouseMove ? "AllowMouseMove" : "DisallowMouseMove")
              << std::endl;
  }
}
