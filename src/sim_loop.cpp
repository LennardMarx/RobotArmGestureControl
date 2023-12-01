#include "../include/sim_loop.h"
#include "../include/hand_gesture_detection.h"

SimLoop::SimLoop() : pendulum(Pendulum(0, 0)) {}
SimLoop::~SimLoop() {}

void SimLoop::run()
{
    chdir(SDL_GetBasePath());

    HandGestures handGestures;
    int fingerDistance = 0;

    while (!helperVars.getQuit())
    {
        std::array<std::array<int, 2>, 21> lms;
        lms = handGestures.estimateLandmarks();

        frameStart = SDL_GetTicks(); // limit framerate (see end of while loop)

        // check for Keyboard inputs;
        eventChecks.checkEvents(helperVars, pendulumDynamics);

        // pause the game
        if (pause)
        {
            // do nothing
        }
        else
        {
            // reset
            if (helperVars.getReset() == true)
            {
                pendulum.setStates({ 0, 0, 0, 0 });
                helperVars.toggleReset();
                //reset = false;
                // ALSO HAVE TO RESET TURN COUNT!!!
            }

            // mouse position
            mouseState = SDL_GetMouseState(&x, &y);

            // coordinates to window center
            x = x - window_width / 2;
            y = y - window_height / 2;

            // FINGER ROTATION AROUND CENTER NOT COUNTED!
            // finger position (between index and thumb)
            x = (lms[8][0] + lms[4][0]) / 2;
            y = (lms[8][1] + lms[4][1]) / 2;

            fingerDistance = handGestures.fingerDistance(lms);
            // std::cout << fingerDistance << std::endl;
            if (fingerDistance > 75 && fingerDistance != 0) {
                pendulumDynamics.setControllerState(false);
            }
            else if (fingerDistance < 75 && fingerDistance != 0) {
                pendulumDynamics.setControllerState(true);
            }

            // converting pixels to endeffector position in meters
            x_conv = x * 1 / (l1 + l2); // total length of robot arm over total length in pixels
            y_conv = -y * 1 / (l1 + l2);

            // checking if desired position is inside workspace
            if (sqrt(x_conv * x_conv + y_conv * y_conv) < 1)
            {
                xd = x_conv;
                yd = y_conv; // flipping y for IK assumptions
            }
            // when mouse is outside of workspace
            else if (sqrt(x_conv * x_conv + y_conv * y_conv) >= 1)
            {
                // calculate position on unit circle (total robot length here = 1!!) -> multiply by total length
                double th1 = atan(y_conv / x_conv);
                xd = 0.99999 * cos(th1); // can't handle exactly 1
                yd = 0.99999 * sin(th1);
                if (x_conv < 0)
                {
                    xd = -xd; yd = -yd; // flipping for quadrant 2 and 3
                }
            }

            // integration
            for (int i = 0; i < 10; ++i) // two integration steps per frame
            {
                pendulumDynamics.setReceivedInputs(pendulumDynamics.inverseKinematics({ xd, yd }));
                // seperate controller? (maybe seperate IK?)
                pendulumDynamics.setReceivedStates(pendulum.getStates());
                pendulumDynamics.rungeKutta();
                pendulum.setStates(pendulumDynamics.getUpdatedStates());
            }

            // calculate coordinates of links (relative angles)
            x1 = x0 + l1 * sin(pendulum.getStates().at(0));
            y1 = y0 + l1 * cos(pendulum.getStates().at(0));
            x2 = x1 + l2 * sin(pendulum.getStates().at(0) + pendulum.getStates().at(1));
            y2 = y1 + l2 * cos(pendulum.getStates().at(0) + pendulum.getStates().at(1));
        }
        helperVars.getTrajectory().push_back({ x2, y2 }); // store endeffector position in trajectory

        // rendering screen
        ui.clear(); // clears screen

        int width = 10; // half width of the link
        // angles of the short side of the link to the origin (makes it a bit easier to calculate)
        double ang1 = pi / 2 - pendulum.getStates().at(0);
        double ang2 = pi / 2 - (pendulum.getStates().at(0) + pendulum.getStates().at(1));

        // drawing the links
        ui.drawTiltedRectangle(x0, y0, x1, y1, ang1, width);
        ui.drawTiltedRectangle(x1, y1, x2, y2, ang2, width);

        // draw trajectory (intensity dependend on recency), define trajectory length
        if (helperVars.getTrajOn())
        {
            ui.drawTrajectory(helperVars.getTrajectory(), 200);
        }
        ui.setDrawColor(255, 255, 255, 150);

        ui.drawHandLandmarks(lms);
        ui.present(); // shows rendered objects

        frameCount += 1; // count Frame

        // frame time to limit FPS
        frameTime = SDL_GetTicks() - frameStart;
        if (frameDelay > frameTime)
        {
            SDL_Delay(frameDelay - frameTime);
        }
    }
}