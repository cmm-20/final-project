#pragma once


#include <gui/application.h>
#include <gui/shader.h>
#include <gui/renderer.h>
#include <gui/light.h>
#include <gui/camera.h>
#include <gui/ImGuizmo.h>
#include <utils/logger.h>
#include <robot/Robot.h>
#include <ode/ODERBEngine.h>



/**
    Space: run/pause sim
    Click on robot: highlight the body part, print out the name.
*/

class App : public Basic3DAppWithShadows {
public:
    App(const char* title = "CRL Playground - Creature Locomotion app", std::string iconPath = CRL_DATA_FOLDER"/icons/icon.png")
        : Basic3DAppWithShadows(title, iconPath) {

        camera = TrackingCamera(5);
        camera.aspectRatio = float(width) / height;
        camera.rotAboutUpAxis = -0.75;
        camera.rotAboutRightAxis = 0.5;

        light.s = 0.03f;
        shadowbias = 0.0f;

        glEnable(GL_DEPTH_TEST);

        showConsole = true;
        automanageConsole = true;
        Logger::maxConsoleLineCount = 10;
        consoleHeight = 225;

        odeRbEngine->loadRBsFromFile(CRL_DATA_FOLDER "/environment/Ground.rbs");

        robot.showMeshes = false;
        robot.showSkeleton = true;

        robot.setRootState(P3D(0, 0.5, 0), Quaternion::Identity());

        // set all the joints to position mode
        for (int i = 0; i < robot.getJointCount(); i++) {
            robot.getJoint(i)->controlMode = RBJointControlMode::POSITION_MODE;
        }

        this->targetFramerate = 30;
        this->limitFramerate = true;

        for(int i = 0; i < MY_PLOT_N; i++)
            myPlotValues[i] = 0;
    }

    virtual ~App() override {
    }

    virtual void resizeWindow(int width, int height) override {
        camera.aspectRatio = float(width) / height;
        return Application::resizeWindow(width, height);
    }

    bool mouseMove(double xpos, double ypos) override {
        P3D rayOrigin;
        V3D rayDirection;
        camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);
        Ray mouseRay(rayOrigin, rayDirection);

        if (mouseState.dragging == false) {
            // this bit of code will check for mouse-ray robot intersections all the time.
            if (selectedRB != NULL)
                selectedRB->rbProps.selected = false;

            P3D selectedPoint;

            selectedRB = robot.getFirstRBHitByRay(mouseRay, selectedPoint, false, true);

            if (selectedRB != NULL) {
                selectedRB->rbProps.selected = true;
            }
        }
        else {
        }

        camera.processMouseMove(mouseState, keyboardState);
        return true;
    }

    bool mouseButtonReleased(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (selectedRB) {
                selectedRB->rbProps.selected = false;
                selectedRB = nullptr;
            }
        }
        return true;
    }

    bool mouseButtonPressed(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT) {
            if (selectedRB != NULL) {
                if (selectedRB->pJoint == NULL)
                    Logger::consolePrint("Clicked on BodyLink %s (root)\n", selectedRB->name.c_str());
                else
                    Logger::consolePrint("Clicked on BodyLink %s (parent joint is %s with id %d))\n", selectedRB->name.c_str(), selectedRB->pJoint->name.c_str(), selectedRB->pJoint->jIndex);
            }
        }

        return true;
    }

    bool scrollWheel(double xoffset, double yoffset) override {
        camera.processMouseScroll(xoffset, yoffset);
        return true;
    }

    void computeAndApplyControlInputs(double dt) {
        simTime += dt;

        // let's say we want the robot to go from the zero angle configuration
        // (as it is loaded) to the default pose in 2 seconds:
        double interpVal = simTime / 2.0;
        boundToRange(&interpVal, 0, 1.0);

        for(const auto joint : robot.jointList)
            joint->desiredControlSignal = interpVal * joint->defaultJointAngle;
    }

    void process() override {
        if (appIsRunning == false)
            return;

        // we need to do enough work here until the simulation time is caught up
        // with the display time...
        double tmpT = 0;
        while (tmpT < 1.0 / targetFramerate) {
            tmpT += dt;

            computeAndApplyControlInputs(dt);

            odeRbEngine->step(dt);
        }

        light.target.x() = robot.root->state.pos.x;
        light.target.z() = robot.root->state.pos.z;

        camera.target.x = robot.root->state.pos.x;
        camera.target.z = robot.root->state.pos.z;
    }

    virtual void drawAuxiliaryInfo() {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawFPS();

        drawConsole();

        drawImGui();

        ImGui::EndFrame();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    // objects drawn with a shadowMapRenderer (during shadow pass) will cast a shadow
    virtual void drawShadowCastingObjects() override {
        robot.draw(shadowMapRenderer);
    }

    // objects drawn with a shadowShader (during the render pass) will have shadows cast on them
    virtual void drawObjectsWithShadows() override {
        ground.draw(shadowShader, V3D(0.6, 0.6, 0.8));
    }

    // objects drawn with basic shadowShader (during the render pass) will not have shadows cast on them
    virtual void drawObjectsWithoutShadows() override {
        robot.draw(basicShader);
    }

    virtual bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            appIsRunning = !appIsRunning;
            return true;
        }
        return false;
    }

    // draws the UI using ImGui. you can add your own UI elements (see below)
    virtual void drawImGui(){

        using namespace ImGui;

        SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
        Begin("Main Menu");

        Text("Play:");
        SameLine();
        ToggleButton("Play App", &appIsRunning);

        if (appIsRunning == false) {
            SameLine();
            if (ArrowButton("tmp", ImGuiDir_Right)) {
                appIsRunning = true;
                process();
                appIsRunning = false;
            }
        }

        Checkbox("Draw Console", &showConsole);

        if (TreeNode("Draw options...")) {
            Checkbox("Draw Meshes", &robot.showMeshes);
            Checkbox("Draw Skeleton", &robot.showSkeleton);
            Checkbox("Draw Joint Axes", &robot.showJointAxes);
            Checkbox("Draw Joint Limits", &robot.showJointLimits);
            Checkbox("Draw Collision Primitives", &robot.showCollisionSpheres);
            Checkbox("Draw MOI box", &robot.showMOI);

            TreePop();
        }

        // add your own UI elements for example like this:
        if(CollapsingHeader("my UI")){
            Indent();

            Checkbox("my bool", &myBool);
            if(myBool){
                SameLine();
                Text(" = true");
            }

            InputDouble("my double", &myDouble);
            // make sure to always unique labels
            // SliderScalar("my double", ... ); // this would not work!
            double min = 0, max = 3;
            if(SliderScalar("my double 2", ImGuiDataType_Double, &myDouble2, &min, &max))
                Logger::consolePrint("myDouble changed to '%f'", myDouble2);

            if(Button("multiply doubles"))
                Logger::consolePrint("result is: %f", myDouble*myDouble2);

            // also works with Eigen vectors
            InputScalarN("my vector", ImGuiDataType_Double, myVector3d.data(), myVector3d.size());

            // make a plot
            PlotLines("my plot", myPlotValues, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {0, 200});
            // You probably would want to put the following lines in the process() function.
            // I put it here so it's all in one place.
            myPlotValues[myPlotCounter] = myDouble2 * myDouble;
            myPlotCounter = (myPlotCounter+1) % MY_PLOT_N;

            Text("guizmo:");
            Checkbox("enabled", &guizmoEnabled);
            InputScalarN("position", ImGuiDataType_Double, guizmoPosition.data(), 3);

            Unindent();
        }

        // example code on how to use the ImGuizmo
        if(guizmoEnabled)
        {
            ImGuizmo::BeginFrame();
            ImGuiIO& io = ImGui::GetIO();
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

            // we use it in translate mode and need to provide the corresponding
            // transformation matrix to ImGuizmo ...
            auto transform = glm::translate(glm::mat4(1.f), toGLM(guizmoPosition));
            ImGuizmo::Manipulate(glm::value_ptr(camera.getViewMatrix()), glm::value_ptr(camera.getProjectionMatrix()), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(transform));

            // ... and thus after manipulation, we extract the changed position
            // from the transformation matrix.
            guizmoPosition = Vector3d(
                transform[3][0],
                transform[3][1],
                transform[3][2]
                );
        }

        ImGui::End();

        // start a new ImGui window like this:
        // ImGui::Begin("my window");
        // ImGui::Button("hello!");
        // ...
        // ImGui::End();

    }

    virtual bool drop(int count, const char** fileNames) override {
        return true;
    }

public:
    SimpleGroundModel ground;   // model to draw the ground

    // the simulation engine
    crl::sim::ODERBEngine* odeRbEngine = new crl::sim::ODERBEngine();

    // the robot to load. uncomment/comment to change robot model
    Robot robot =
        Robot(odeRbEngine, CRL_DATA_FOLDER"/robots/simple/hex.rbs");
//      Robot(odeRbEngine, CRL_DATA_FOLDER"/robots/simple/dog.rbs");

    RobotRB* selectedRB = NULL; // robot rigid body selected by mouse, = NULL when nothing selected
    bool appIsRunning = false;

    double dt = 1 / 120.0;      // time step for the simulation
    double simTime = 0;         // current simulation time

    // example variables for how to use ImGui
    bool myBool = false;
    double myDouble = 1;
    double myDouble2 = 2;
    Vector3d myVector3d = {1,2,3};
    int myPlotCounter = 0;
    const static int MY_PLOT_N = 100;
    float myPlotValues[MY_PLOT_N];

    // example for a 3d guizmo
    bool guizmoEnabled = false;
    Vector3d guizmoPosition = Vector3d{0,0,0};

};
