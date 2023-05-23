
``` C
// // If inside outer circle
          // if (in_abs < out_radius)
          // { // if inside in circle
          //   if (in_abs < in_radius)
          //   {
          //     // if (in_abs < 0.5)
          //     // {
          //     //   aux_x = x_pos - obs_x;
          //     //   aux_y = y_pos - obs_y;
          //     // }
          //     // else
          //     // {
          //     //   ref = ts.track_bearing;
          //     // }

          //     inf("Inside circle radius");
          //     // Is inside obstacle circle
          //     aux_x = in_x - obs_x;
          //     aux_y = in_y - obs_y;
          //     ref = atan2(aux_y, aux_x); // Vector orientation
          //   }
          //   else // flowing through the ring
          //   {
          //     inf("On outer circle radius");

          //     centerX = obs_x - x_pos;
          //     centerY = obs_y - y_pos;

          //     // If past upper half of the circle
          //     if (centerX == 0 || centerY > 0)
          //     {
          //       theta = atan(centerY / centerX);

          //       if (laranja)
          //       {
          //         inf("Inside laranja lol");
          //         if (centerX < 0 && centerY > 0)
          //         {
          //           aux_x = cos(theta - DUNE::Math::c_pi / 2);
          //           aux_y = sin(theta - DUNE::Math::c_pi / 2);
          //         }
          //         else if ((centerX <= 0 && centerY <= 0) || (centerX > 0 && centerY <= 0) || (centerX >= 0 && centerY > 0))
          //         {
          //           aux_x = cos(theta + DUNE::Math::c_pi / 2);
          //           aux_y = sin(theta + DUNE::Math::c_pi / 2);
          //         }
          //       }
          //       else
          //       {
          //         if ((centerX > 0 && centerY <= 0) || (centerX <= 0 && centerY <= 0) || (centerX < 0 && centerY > 0))
          //         {
          //           aux_x = cos(theta + DUNE::Math::c_pi / 2);
          //           aux_y = sin(theta + DUNE::Math::c_pi / 2);
          //         }
          //         else if (centerX >= 0 && centerY > 0)
          //         {
          //           aux_x = cos(theta - DUNE::Math::c_pi / 2);
          //           aux_y = sin(theta - DUNE::Math::c_pi / 2);
          //         }
          //       }
          //     }
          //   }
          // }
```