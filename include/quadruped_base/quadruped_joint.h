/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef QUADRUPED_JOINT_H
#define QUADRUPED_JOINT_H

namespace champ
{
    class Joint
    {
        float x_; 
        float y_; 
        float z_; 

        float roll_; 
        float pitch_; 
        float yaw_;

        float theta_;

        public:
            Joint( float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y):
                x_(pos_x), 
                y_(pos_y),
                z_(pos_z),
                roll_(or_r),
                pitch_(or_p),
                yaw_(or_y),
                theta_(0)
            { 
            } 

            float theta()
            { 
                return theta_; 
            }

            void theta(float angle)
            { 
                theta_ = angle; 
            }

            float x()
            {
                return x_;
            }

            float y()
            {
                return y_;
            }

            float z()
            {
                return z_;
            }

            float roll()
            {
                return roll_;
            }

            float pitch()
            {
                return pitch_;
            }

            float yaw()
            {
                return yaw_;
            }
    };
}
#endif