
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
            enum MAX_MACRO
            {
                OBS_MAX_NUMBER = 6
            };
            // int final obs_max_number = 6;
            double pos[MAX_MACRO::OBS_MAX_NUMBER][2] = {0}; // x and y
            //! In Neptus coordinates
            void add_obstacle(double x, double y)
            {

                for (int i = 0; i < MAX_MACRO::OBS_MAX_NUMBER; i++)
                {
                    std::cout << "Obstacle pos " << i << std::endl;
                    if (pos[i][0] == 100 && pos[i][1] == 100)
                    {
                        pos[i][0] = x;
                        pos[i][1] = y;
                        printf("Added object to [%d] as %.2f, %.2f\n", i, pos[i][0], pos[i][1]);
                        return;
                    }
                    std::cout << "Can't add obstacle, already filled" << std::endl;
                }
            }

            //! returns the index of the closest object pos
            int closest_object(double my_x, double my_y)
            {
                int index = 0;
                double abs_min = 30;
                for (int i = 0; i < MAX_MACRO::OBS_MAX_NUMBER; i++)
                {
                    if (pos[i][0] != 100 || pos[i][1] != 100)
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

            ObstacleInterface()
            {
                // Pos initializer. GOing to 30 to have a compare thing
                for (int i = 0; i < MAX_MACRO::OBS_MAX_NUMBER; i++)
                {
                    pos[i][0] = 100; // x
                    pos[i][1] = 100; // y
                }
            }
        };
    }
}

#endif