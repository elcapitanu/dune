
#ifndef DRIVER_HPP_INCLUDED_
#define DRIVER_HPP_INCLUDED_

#include <DUNE/DUNE.hpp>

namespace Control
{

    namespace Path
    {
        using DUNE_NAMESPACES;

        class ObstacleInterface
        {
        public:
            DUNE::Tasks::Task *m_task;
            float in_radius;

            enum MAX_MACRO
            {
                OBS_MAX_NUMBER = 100
            };
            // int final obs_max_number = 6;
            double pos[MAX_MACRO::OBS_MAX_NUMBER][2] = {0}; // x and y

            ObstacleInterface(DUNE::Tasks::Task *task, float radius)
            {
                m_task = task;
                in_radius = radius;
                // Pos initializer. GOing to 30 to have a compare thing
                for (int i = 0; i < MAX_MACRO::OBS_MAX_NUMBER; i++)
                {
                    pos[i][0] = 1000; // x
                    pos[i][1] = 1000; // y
                }
            }

            //! In Neptus coordinates
            void add_obstacle(double x, double y)
            {

                for (int i = 0; i < MAX_MACRO::OBS_MAX_NUMBER; i++)
                {

                    double in_x = abs(pos[i][0] - x);
                    double in_y = abs(pos[i][1] - y);

                    double in_abs = sqrt(pow(in_x, 2) + pow(in_y, 2));

                    if (in_abs < in_radius * 0.9)
                    {
                        m_task->err("Obstacle not added, too close to already existing object");
                        return;
                    }

                    std::cout << "Obstacle pos " << i << std::endl;
                    if (pos[i][0] == 1000 && pos[i][1] == 1000)
                    {
                        pos[i][0] = x;
                        pos[i][1] = y;
                        printf("Added object to [%d] as %.2f, %.2f\n", i, pos[i][0], pos[i][1]);
                        return;
                    }
                    // std::cout << "Can't add obstacle, already filled" << std::endl;
                }
            }

            //! returns the index of the closest object pos
            int closest_object(double my_x, double my_y)
            {
                int index = 0;
                double abs_min = 30;
                for (int i = 0; i < MAX_MACRO::OBS_MAX_NUMBER; i++)
                {
                    if (pos[i][0] != 1000 || pos[i][1] != 1000)
                    {
                        double in_x = abs(pos[i][0] - my_x);
                        double in_y = abs(pos[i][1] - my_y);

                        double in_abs = sqrt(pow(in_x, 2) + pow(in_y, 2));

                        // printf("Pos(X, Y): %.2f, %.2f -> ABS[%d]: %.2f\n", my_x, my_y, i, in_abs);

                        if (in_abs < abs_min)
                        {
                            index = i;
                            abs_min = in_abs;
                        }
                    }
                }

                return index;
            }
        };
    }
}

#endif