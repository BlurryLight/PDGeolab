#pragma once
// Modified from LearnOpenGL.com
#include <fstream>
// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
// clang-format on
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace PD {

using uint = unsigned int;

// Defines several possible options for camera movement. Used as abstraction
// to stay away from window-system specific input methods
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT };

// Default camera values
constexpr static float YAW = -90.0f;
constexpr static float PITCH = 0.0f;
constexpr static float SPEED = 2.5f;
constexpr static float SENSITIVITY = 0.1f;
constexpr static float ZOOM = 45.0f;

// classes
class Camera;
class Mesh;
class Shader;
struct Vertex;
struct Texture;
class Model;

class Mesh {
public:
  // mesh Data
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;
  unsigned int VAO;

  // constructor
  Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
       std::vector<Texture> textures);

  // render the mesh
  void Draw(Shader &shader);

private:
  // render data
  unsigned int VBO, EBO;

  // initializes all the buffer objects/arrays
  void setupMesh();
};
class Model {
public:
  // model data
  std::vector<Texture>
      textures_loaded; // stores all the textures loaded so far, optimization to
                       // make sure textures aren't loaded more than once.
  std::vector<Mesh> meshes;
  std::string directory;
  bool gammaCorrection;

  // constructor, expects a filepath to a 3D model.
  Model(std::string const &path, bool gamma = false);

  // draws the model, and thus all its meshes
  void Draw(Shader &shader);

private:
  // loads a model with supported ASSIMP extensions from file and stores the
  // resulting meshes in the meshes vector.
  void loadModel(std::string const &path);

  // processes a node in a recursive fashion. Processes each individual mesh
  // located at the node and repeats this process on its children nodes (if
  // any).
  void processNode(aiNode *node, const aiScene *scene);

  Mesh processMesh(aiMesh *mesh, const aiScene *scene);

  // checks all material textures of a given type and loads the textures if
  // they're not loaded yet. the required info is returned as a Texture struct.
  std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type,
                                            std::string typeName);
};

// funcs
unsigned int TextureFromFile(const char *path,
                             const std::string &directory = "",
                             bool gamma = false, bool flip = false);
void renderQuad();
void renderCube();
void renderSphere();
inline float lerp(float a, float b, float t) { return a * (1 - t) + b * t; }

inline float get_random_float(float min=0.0f,float max=1.0f)
{
  static std::mt19937 generator;
  std::uniform_real_distribution<float> dis(min,max);
  return dis(generator);
}

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
  // Camera Attributes
  glm::vec3 Position;
  glm::vec3 Front;
  glm::vec3 Up;
  glm::vec3 LookAt;
  glm::vec3 Right;
  glm::vec3 WorldUp;
  // Euler Angles
  float Yaw;
  float Pitch;
  // Camera options
  float MovementSpeed;
  float MouseSensitivity;
  float Zoom;

  // Constructor with vectors
  Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
         glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
         glm::vec3 lookat = glm::vec3(0.0f, 0.0f, -1.0f), float fov = ZOOM,
         float yaw = YAW, float pitch = PITCH);

  // Returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 GetViewMatrix();

  // Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
  void ProcessKeyboard(Camera_Movement direction, float deltaTime);

  // Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
  void ProcessMouseMovement(float xoffset, float yoffset,
                            GLboolean constrainPitch = true);

  // Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
  void ProcessMouseScroll(float yoffset);

private:
  // Calculates the front vector from the Camera's (updated) Euler Angles
  void updateCameraVectors();
};

class Shader
{
public:
  unsigned int ID;
  // constructor generates the shader on the fly
  // ------------------------------------------------------------------------
  Shader(const char *vertexPath, const char *fragmentPath,
         const char *geometryPath = nullptr);
  Shader(const std::string &vertexPath, const std::string &fragmentPath)
      : Shader(vertexPath.c_str(), fragmentPath.c_str()) {}
  Shader(const std::string &vertexPath, const std::string &fragmentPath,
         const std::string &geometryPath)
      : Shader(vertexPath.c_str(), fragmentPath.c_str(), geometryPath.c_str()) {
  }
  // activate the shader
  // ------------------------------------------------------------------------
  void use() const;
  // utility uniform functions
  // ------------------------------------------------------------------------
  void setBool(const std::string &name, bool value) const;
  // ------------------------------------------------------------------------
  void setInt(const std::string &name, int value) const;
  // ------------------------------------------------------------------------
  void setFloat(const std::string &name, float value) const;
  // ------------------------------------------------------------------------
  void setVec2(const std::string &name, const glm::vec2 &value) const;
  void setVec2(const std::string &name, float x, float y) const;
  // ------------------------------------------------------------------------
  void setVec3(const std::string &name, const glm::vec3 &value) const;
  void setVec3(const std::string &name, float x, float y, float z) const;
  // ------------------------------------------------------------------------
  void setVec4(const std::string &name, const glm::vec4 &value) const;
  void setVec4(const std::string &name, float x, float y, float z, float w);
  // ------------------------------------------------------------------------
  void setMat2(const std::string &name, const glm::mat2 &mat) const;
  // ------------------------------------------------------------------------
  void setMat3(const std::string &name, const glm::mat3 &mat) const;
  // ------------------------------------------------------------------------
  void setMat4(const std::string &name, const glm::mat4 &mat) const;

private:
  // utility function for checking shader compilation/linking errors.
  // ------------------------------------------------------------------------
  void checkCompileErrors(GLuint shader, std::string type);
};

struct Vertex {
  // position
  glm::vec3 Position;
  // normal
  glm::vec3 Normal;
  // texCoords
  glm::vec2 TexCoords;
  // tangent
  glm::vec3 Tangent;
  // bitangent
  glm::vec3 Bitangent;
};

struct Texture {
  unsigned int id;
  std::string type;
  std::string path;
};

} // namespace PD
