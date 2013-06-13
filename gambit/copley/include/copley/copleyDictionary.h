#ifndef __COPLEY_DICTIONARY_H__ 
#define __COPLEY_DICTIONARY_H__ 

#define MFG_STATUS_REGISTER                                    0x1002  //pg 56 

#define HIGH_RESOLUTION_TIMESTAMP                              0x1013  //pg 45

#define RX_PDO_COM_PARAM_0                                     0x1400  //pg 32
#define RX_PDO_COM_PARAM_1                                     0x1401  //pg 32
#define RX_PDO_COM_PARAM_2                                     0x1402  //pg 32
#define RX_PDO_COM_PARAM_3                                     0x1403  //pg 32
#define RX_PDO_COM_PARAM_4                                     0x1404  //pg 32
#define RX_PDO_COM_PARAM_5                                     0x1405  //pg 32
#define RX_PDO_COM_PARAM_6                                     0x1406  //pg 32
#define RX_PDO_COM_PARAM_7                                     0x1407  //pg 32

#define RX_PDO_MAPPING_PARAMETERS0                             0x1600  //pg 33
#define RX_PDO_MAPPING_PARAMETERS1                             0x1601  //pg 33
#define RX_PDO_MAPPING_PARAMETERS2                             0x1602  //pg 33
#define RX_PDO_MAPPING_PARAMETERS3                             0x1603  //pg 33
#define RX_PDO_MAPPING_PARAMETERS4                             0x1604  //pg 33
#define RX_PDO_MAPPING_PARAMETERS5                             0x1605  //pg 33
#define RX_PDO_MAPPING_PARAMETERS6                             0x1606  //pg 33
#define RX_PDO_MAPPING_PARAMETERS7                             0x1607  //pg 33

#define TX_PDO_COM_PARAM_0                                     0x1800  //pg 34
#define TX_PDO_COM_PARAM_1                                     0x1801  //pg 34
#define TX_PDO_COM_PARAM_2                                     0x1802  //pg 34
#define TX_PDO_COM_PARAM_3                                     0x1803  //pg 34
#define TX_PDO_COM_PARAM_4                                     0x1804  //pg 34
#define TX_PDO_COM_PARAM_5                                     0x1805  //pg 34
#define TX_PDO_COM_PARAM_6                                     0x1806  //pg 34
#define TX_PDO_COM_PARAM_7                                     0x1807  //pg 34

#define TX_PDO_MAPPING_PARAMETERS0                             0x1a00  //pg 33
#define TX_PDO_MAPPING_PARAMETERS1                             0x1a01  //pg 33
#define TX_PDO_MAPPING_PARAMETERS2                             0x1a02  //pg 33
#define TX_PDO_MAPPING_PARAMETERS3                             0x1a03  //pg 33
#define TX_PDO_MAPPING_PARAMETERS4                             0x1a04  //pg 33
#define TX_PDO_MAPPING_PARAMETERS5                             0x1a05  //pg 33
#define TX_PDO_MAPPING_PARAMETERS6                             0x1a06  //pg 33
#define TX_PDO_MAPPING_PARAMETERS7                             0x1a07  //pg 33

#define IP_MOVE_SEGMENT_COMMAND                                0x2010  //pg 187

#define TRAJECTORY_BUFFER_FREE_COUNT                           0x2011  //pg 188
#define TRAJECTORY_BUFFER_STATUS                               0x2012  //pg 189

#define USER_PEAK_CURRENT_LIMIT                                0x2110  //pg 126
#define USER_CONTINUOUS_CURRENT_LIMIT                          0x2111  //pg 126
#define USER_PEAK_CURRENT_LIMIT_TIME                           0x2112  //pg 126

#define TRACKING_ERROR_WINDOW                                  0x2120  //pg 62

#define LATCHING_FAULT_STATUS_REGISTER                         0x2183  //pg 64

#define DESIRED_STATE                                          0x2300  //pg 60 

#define CONTROL_WORD                                           0x6040  //pg 54 
#define STATUS_WORD                                            0x6041  //pg 55 


#define MODE_OF_OPERATION                                      0x6060  //pg 59 
#define MODE_OF_OPERATION_DISPLAY                              0x6061  //pg 59 

#define POSITION_ACTUAL_VALUE                                  0x6064  //pg 114 

#define TARGET_POSITION                                        0x607a  //pg 175 

#define HOMING_METHOD                                          0x6098  //pg 157 

#define INTERPOLATION_SUBMODE_SELECT                           0x60c0  //pg 189 

#define INTERPOLATION_DATA_RECORD                              0x60c1  //pg 190 

#define INTERPOLATION_CONSTANT_TIME                            0x60c2  //pg 190

#endif
