////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Gokul Subramanian, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Gokul Subramanian
/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef sbpl_math_h
#define sbpl_math_h

#include <math.h>
#include <vector>

/* Basic Operations */
void multiply(double*, double*, int, int, double*, int);
void scalar_multiply(double*, double*, int, int, double);
void equate(double*, double*, int, int);
void matrix_add(double*, double*, double*, int, int);
void subtract(double*, double*, double*, int, int);
void transpose(double*, double*, int, int);

/* Vector Operations */
double dot_product(double*, double*, int);
void cross_product(double*, double*, double*);
double vect_norm(double*, int);
double vect_divide(double*, double*, int);
bool check_equality(double*, double*, int);

/* 3D Geometry */
void create_rotation_matrix(double*, double, double, double);
void rotate_vector(double (&result)[3], double*, double*, double);
double distance_between(double*,  double*, int);
double distance_between(std::vector<double>, double*, int);

#endif
