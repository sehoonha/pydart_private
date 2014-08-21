%module pydart_api

%{
  #define SWIG_FILE_WITH_INIT
  #include "pydart_api.h"
%}

/* Include the NumPy typemaps library */
%include "numpy.i"

%init %{
  import_array();
%}

/* Typemap for the sum_list(double* input_array, int length) C/C++ routine */
%apply (double* IN_ARRAY1, int DIM1) {(double* inpose, int ndofs)};
%apply (double* IN_ARRAY1, int DIM1) {(double* intorque, int ndofs)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outpose, int ndofs)};
%apply (double ARGOUT_ARRAY1[ANY]) {(double outv3[3])};
%apply (double* INPLACE_ARRAY2, int DIM1, int DIM2) {(double* array2, int nrows, int ncols)};


%include "pydart_api.h"
