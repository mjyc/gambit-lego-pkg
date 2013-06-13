#ifndef __COPLEY_ERRORS_H__
#define __COPLEY_ERRORS_H__

#define ERROR_VERBOSITY_LEVEL 1

#define RETURN_ON_ERROR( FNCALL )  {  int qq_err; \
                                      qq_err = FNCALL; \
                                      if ((ERROR_VERBOSITY_LEVEL-0 > 0) && qq_err) { \
                                        printf("error %d in %s at %d\n", qq_err, __FILE__, __LINE__); \
                                      } \
                                      if ((ERROR_VERBOSITY_LEVEL-0 > 1) && !qq_err) {\
                                        printf("success in %s at %d\n", __FILE__, __LINE__); \
                                      } \
                                      if (qq_err) { \
                                        return qq_err; \
                                      } \
                                    }  

#define ERROR_NONE              0 
#define ERROR_FAILED_SOCKET     1
#define ERROR_FAILED_SETSOCKOPT 2
#define ERROR_TIMEOUT           3
#define ERROR_UNEXPECTED_COBID  4
#define ERROR_BAD_SCS           5
#define ERROR_NOT_EXPEDITED     6 
#define ERROR_BAD_INDEX         7
#define ERROR_BAD_SUBINDEX      8
#define ERROR_BAD_DATA          9 
#define ERROR_BAD_CCS           10 
#define ERROR_BAD_LEN           11 
#define ERROR_TOO_FAST          12
#define ERROR_TOO_SLOW          13
#define ERROR_HOME_NOT_FOUND    14
#define ERROR_MOVE_NOT_ACTIVE   15

const char *copleyGetErrorString(int code);

#endif

