#Input configuration info
MIN_OBJSIZE_FRAC = .0033
MIN_HANDSIZE_FRAC = .002

camAlpha = 525 # asus
#camAlpha = 570 # kinect
videoWidth = 320
videoHeight = 240
cam_x =  319 # transformation applied to unchanged file
cam_y = 239

cameraConfigFile = "./SamplesConfig.xml"

showFPS = True

HAND_THRESHOLD_HIGH = 50     # in mm
HAND_THRESHOLD_LOW = 5     # in mm

# shared parameters
BACKTRACK_LIM = 0
DISPLAY_FLAG = True
saveWorldsAfter = False
saveWorldsConcurrently = False

WORLD_SNAPSHOT_LIM = 1
NOMINAL_MIN_DEPTH = 500
NOMINAL_MAX_DEPTH = 2000
VNI_GRASPED_SIM_THRESH = .83
PROBATION_LENGTH = 6
VNI_DIFF_THRESH = .88
OCC_DIFF_THRESH = .8
DEPTH_NOISE_MARGIN = 10
OBJ_DEPTH_NOISE_MARGIN = 10
HAND_SIZE = 80
HAND_SMOOTHING_PARAM = 2
SLEEVES = True
HAND_LEN_PX = 50

# Live-specific
liveNTableFrames = 20 # number of frames used for compute table depth



# Mike's
DEFAULT_PARAMS_PATH = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params";
