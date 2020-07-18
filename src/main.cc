

// GLAD
#include "glsupport.h"
#include "resource_path_searcher.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <fstream>
#include <glad/glad.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
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

void prim_mst(const MyMesh &mesh, int begin_node) {
  std::unordered_map<MyMesh::VHandle, float> dis;
  std::unordered_map<MyMesh::VHandle, MyMesh::VHandle> parent;
  std::unordered_map<MyMesh::VHandle, bool> inMST;
  //  dis.assign(mesh.points()->size(), std::numeric_limits<float>::max());
  for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
    dis.emplace(*v_it, std::numeric_limits<float>::max());
    inMST.emplace(*v_it, false);
  }
  auto cmp = [](std::pair<MyMesh::VHandle, float> a,
                std::pair<MyMesh::VHandle, float> b) {
    return a.second > b.second;
  };

  std::priority_queue<std::pair<MyMesh::VHandle, float>,
                      std::vector<std::pair<MyMesh::VHandle, float>>,
                      decltype(cmp)>
      pq(cmp);
  auto v_it = mesh.vertex_handle(begin_node);
  dis[v_it] = 0;
  parent[v_it] = *(mesh.vertices_end());
  pq.push({v_it, 0});

  while (!pq.empty()) {
    auto tmp = pq.top();
    pq.pop();
    if (inMST[tmp.first])
      continue; // avoid infinite loop
    inMST[tmp.first] = true;
    for (auto vvit = mesh.cvv_iter(tmp.first); vvit.is_valid(); vvit++) {
      //      std::cout << "vvit:" << mesh.point(*vvit) << std::endl;
      float weight = (mesh.point(*vvit) - mesh.point(tmp.first)).norm();
      if (inMST[*vvit] == false && dis[*vvit] > weight) {
        dis[*vvit] = weight;
        pq.push({vvit, weight});
        parent[*vvit] = tmp.first;
      }
    }
  }
  std::cout << "num of vertices: " << dis.size() << std::endl;
  for (auto vit = mesh.vertices_begin() + 1; vit != mesh.vertices_end();
       vit++) {
    std::printf("%d -> %d ", parent[*vit].idx(), vit->idx());
  }
  std::cout << "end" << std::endl;
}
void dijkstra(const MyMesh &mesh, int begin_node, int end_node) {

  std::unordered_map<MyMesh::VHandle, float> dis;
  std::unordered_map<MyMesh::VHandle, MyMesh::VHandle> parent;
  //  dis.assign(mesh.points()->size(), std::numeric_limits<float>::max());
  for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
    dis.emplace(*v_it, std::numeric_limits<float>::max());
  }
  auto cmp = [](std::pair<MyMesh::VHandle, float> a,
                std::pair<MyMesh::VHandle, float> b) {
    return a.second > b.second;
  };

  uint des = end_node;
  if (des >= dis.size())
    throw std::out_of_range("destination out of vertex indices range!");
  std::priority_queue<std::pair<MyMesh::VHandle, float>,
                      std::vector<std::pair<MyMesh::VHandle, float>>,
                      decltype(cmp)>
      pq(cmp);
  auto v_it = mesh.vertex_handle(begin_node);
  dis[v_it] = 0;
  parent[v_it] = *(mesh.vertices_end());
  pq.push({v_it, 0});

  while (!pq.empty()) {
    auto tmp = pq.top();
    pq.pop();
    for (auto vvit = mesh.cvv_iter(tmp.first); vvit.is_valid(); vvit++) {
      //      std::cout << "vvit:" << mesh.point(*vvit) << std::endl;
      float weight = (mesh.point(*vvit) - mesh.point(tmp.first)).norm();
      if (dis[*vvit] > weight + dis[tmp.first]) {
        dis[*vvit] = weight + dis[tmp.first];
        pq.push({vvit, weight + dis[tmp.first]});
        parent[*vvit] = tmp.first;
        if (vvit->idx() == des)
          goto find_des;
      }
    }
  }
find_des:

  std::cout << "num of vertices: " << dis.size() << std::endl;
  std::vector<uint> path;
  for (auto it = mesh.vertex_handle(des); it != *mesh.vertices_end();) {
    path.push_back(it.idx());
    it = parent[it];
  }
  std::cout << "Path from " << begin_node << " to " << des << std::endl;
  for (auto r = path.rbegin(); r != path.rend(); r++) {
    std::cout << *r << "->";
  }
  std::cout << "end" << std::endl;
}
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
  resourcesPath.add_path(
      (ResourcePathSearcher::root_path / "resources" / "models").u8string());

  Shader shader(resourcesPath.find_path("simple.vert"),
                resourcesPath.find_path("simple.frag"));
  MyMesh mesh;
  // read mesh from stdin
  if (!OpenMesh::IO::read_mesh(
          mesh,
          resourcesPath.find_path(std::vector<std::string>{"cube.obj"}))) {
    return -1;
  }
  //  dijkstra(mesh, 1, 6);
  prim_mst(mesh, 0);
  Model model(resourcesPath.find_path("cube.obj"));
  cam = Camera();
  // main loop
  bool wireframe_mode = false;
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

      if (ImGui::Button("Toggle WireFrame"))
        wireframe_mode = !wireframe_mode;
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();
    }
    ImGui::Render();

    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (wireframe_mode)
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

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
