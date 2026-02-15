#define IMGUI_IMPL_OPENGL_LOADER_GLEW 1
#include <GL/glew.h> // Initialize with glewInit()

// #include "../imgui/libs/gl3w/GL/gl3w.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "../imgui/imgui.hpp"
#include "../imgui/imgui_impl_opengl3.hpp"
#include "../imgui/imgui_impl_sdl.hpp"

#include "imgui-viewer.hpp"
#include "color.hpp"
#include "simple_svg_writer.hpp"
#include "plot-cv.hpp"

std::ofstream logfile("plot-cv.log", std::ios_base::out | std::ios_base::ate);

using namespace std;

//---------------------------------------------------------------------

float
ImageViewer::getGain() {
  return gain;
}

void
ImageViewer::init() {
  // Setup SDL
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
    printf("Error: %s\n", SDL_GetError());
    exit();
  }
  // Decide GL+GLSL versions
#if __APPLE__
  // GL 3.2 Core + GLSL 150
  const char* glsl_version = "#version 150";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS,
                      SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

  // Create window with graphics context
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_WindowFlags window_flags =
      (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  // SDL_Window*
  window = SDL_CreateWindow("OpenCV/ImGUI Viewer",
                            SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED,
                            1280,
                            720,
                            window_flags);
  // SDL_GLContext
  gl_context = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1); // Enable vsync
                             // Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
  bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
  bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
  bool err = gladLoadGL() == 0;
#else
  bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to
                    // requires some form of initialization.
#endif
  if(err) {
    fprintf(stderr, "Failed to initialize OpenGL loader!\n");
    exit();
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer bindings
  ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
  ImGui_ImplOpenGL3_Init(glsl_version);
}

void
ImageViewer::render() {
  // Rendering
  ImGui::Render();
  SDL_GL_MakeCurrent(window, gl_context);
  glViewport(0, 0, (int)io->DisplaySize.x, (int)io->DisplaySize.y);
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  SDL_GL_SwapWindow(window);
}

void
ImageViewer::exit() {
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_GL_DeleteContext(gl_context);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

void
ImageViewer::initContents() {
  clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
}

void
ImageViewer::imshow(std::string frame_name, cv::Mat* frame) {
  if(frame->empty())
    return;

  if(std::any_of(frame_names.cbegin(),
                 frame_names.cend(),
                 [&frame_name](std::string str) -> bool { return frame_name == str; }))
    return;

  frame_names.push_back(frame_name);
  frames.push_back(frame);
}

void
ImageViewer::show(cv::Mat* frame) {
  imshow("image:" + to_string(frames.size()), frame);
}

void
ImageViewer::showMainContents() {
  ImGui::Begin("Main");

  ImGui::SliderFloat("gain", &gain, 0.0f, 2.0f, "%.3f");

  ImGui::Text("%.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate,
              ImGui::GetIO().Framerate);
  ImGui::End();
}

void
ImageViewer::show() {
  int x = 0, y = 0;
  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL2_NewFrame(window);
  ImGui::NewFrame();

  showMainContents();

  // initialize textures
  vector<ImageTexture*> my_textures;
  for(int i = 0; i < frames.size(); i++) {
    ImageTexture* text = new ImageTexture();
    cv::Size s = frames[i]->size();
    if(s.width > 200)
      cv::resize(
          *frames[i], *frames[i], cv::Size(s.width / 2, s.height / 2), 0, 0, cv::INTER_LINEAR);

    text->setImage(frames[i]);
    my_textures.push_back(text);
  }

  // imshow windows
  for(int i = 0; i < frames.size(); i++) {
    ImVec2 size, scaled;
    cv::Mat* frame = frames[i];

    std::string window_name;
    if(frame_names.size() <= i) {
      window_name = "image:" + to_string(i);
    } else {
      window_name = frame_names[i];
    }

    size = my_textures[i]->getSize();

    scaled = size;
    /* scaled[0] /= 2;
     scaled[1] /= 2;
 */
    ImGui::Begin(window_name.c_str());
    ImGui::SetNextWindowPos(ImVec2(x, y));
    ImGui::Image(my_textures[i]->getOpenglTexture(), scaled);
    ImGui::End();

    x += size[0] + 40;
    if(i % 2 == 1) {
      x = 0;
      y += size[1] + 40;
    }
  }

  render();

  // clear resources
  for(int i = 0; i < frames.size(); i++) {
    delete my_textures[i];
  }

  frame_names.clear();
  frames.clear();
  my_textures.clear();
}

bool
ImageViewer::handleEvent() {
  SDL_Event event;
  bool done = false;
  while(SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    if(event.type == SDL_QUIT)
      done = true;
    if(event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
       event.window.windowID == SDL_GetWindowID(window))
      done = true;
  }
  return done;
}

//---------------------------------------------------------------------

int
main(int argc, char* argv[]) {
  ImageViewer gui;

  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    cout << "camera cannot be opened" << endl;
    return (-1);
  }

  js_init(argc, argv);

  // Main loop
  while(!gui.handleEvent()) {
    if(cap.read(imgOriginal)) {
      double scaleFactor = 200.0 / (double)imgOriginal.cols;

      process_image(
          std::bind(&ImageViewer::imshow, &gui, std::placeholders::_1, std::placeholders::_2),
          0);

      /*      cv::resize(imgOriginal, imgOriginal, cv::Size(0, 0), scaleFactor, scaleFactor,
         cv::INTER_LINEAR); cv::resize(imgGrayscale, imgGrayscale, cv::Size(0,
         0),scaleFactor,scaleFactor, cv::INTER_LINEAR); cv::resize(imgCanny, imgCanny,
         cv::Size(0, 0), scaleFactor, scaleFactor, cv::INTER_LINEAR);

      */
      // show halfsize image
      //   gui.imshow("imgRaw", &imgRaw);
      gui.imshow("imgVector", &imgVector);
      gui.imshow("imgOriginal", &imgOriginal);
      //  gui.imshow("imgTemp", &imgTemp);
      gui.imshow("imgGrayscale", &imgGrayscale);
      gui.imshow("imgBlurred", &imgBlurred);
      gui.imshow("imgCanny", &imgCanny);
      gui.imshow("imgMorphology", &imgMorphology);

      // make quartersize image and show
      //
      // gui.imshow("quater", &frame2);

      gui.show();
    }
  }

  cap.release();
  gui.exit();

  return 0;
}
