import { call, define, dlopen, dlsym, RTLD_NOW } from 'ffi';

export const GL_VERSION_1_1 = 1;
export const GL_VERSION_1_2 = 1;
export const GL_VERSION_1_3 = 1;
export const GL_ARB_imaging = 1;
export const GL_FALSE = 0;
export const GL_TRUE = 1;
export const GL_BYTE = 0x1400;
export const GL_UNSIGNED_BYTE = 0x1401;
export const GL_SHORT = 0x1402;
export const GL_UNSIGNED_SHORT = 0x1403;
export const GL_INT = 0x1404;
export const GL_UNSIGNED_INT = 0x1405;
export const GL_FLOAT = 0x1406;
export const GL_2_BYTES = 0x1407;
export const GL_3_BYTES = 0x1408;
export const GL_4_BYTES = 0x1409;
export const GL_DOUBLE = 0x140a;
export const GL_POINTS = 0x0000;
export const GL_LINES = 0x0001;
export const GL_LINE_LOOP = 0x0002;
export const GL_LINE_STRIP = 0x0003;
export const GL_TRIANGLES = 0x0004;
export const GL_TRIANGLE_STRIP = 0x0005;
export const GL_TRIANGLE_FAN = 0x0006;
export const GL_QUADS = 0x0007;
export const GL_QUAD_STRIP = 0x0008;
export const GL_POLYGON = 0x0009;
export const GL_VERTEX_ARRAY = 0x8074;
export const GL_NORMAL_ARRAY = 0x8075;
export const GL_COLOR_ARRAY = 0x8076;
export const GL_INDEX_ARRAY = 0x8077;
export const GL_TEXTURE_COORD_ARRAY = 0x8078;
export const GL_EDGE_FLAG_ARRAY = 0x8079;
export const GL_VERTEX_ARRAY_SIZE = 0x807a;
export const GL_VERTEX_ARRAY_TYPE = 0x807b;
export const GL_VERTEX_ARRAY_STRIDE = 0x807c;
export const GL_NORMAL_ARRAY_TYPE = 0x807e;
export const GL_NORMAL_ARRAY_STRIDE = 0x807f;
export const GL_COLOR_ARRAY_SIZE = 0x8081;
export const GL_COLOR_ARRAY_TYPE = 0x8082;
export const GL_COLOR_ARRAY_STRIDE = 0x8083;
export const GL_INDEX_ARRAY_TYPE = 0x8085;
export const GL_INDEX_ARRAY_STRIDE = 0x8086;
export const GL_TEXTURE_COORD_ARRAY_SIZE = 0x8088;
export const GL_TEXTURE_COORD_ARRAY_TYPE = 0x8089;
export const GL_TEXTURE_COORD_ARRAY_STRIDE = 0x808a;
export const GL_EDGE_FLAG_ARRAY_STRIDE = 0x808c;
export const GL_VERTEX_ARRAY_POINTER = 0x808e;
export const GL_NORMAL_ARRAY_POINTER = 0x808f;
export const GL_COLOR_ARRAY_POINTER = 0x8090;
export const GL_INDEX_ARRAY_POINTER = 0x8091;
export const GL_TEXTURE_COORD_ARRAY_POINTER = 0x8092;
export const GL_EDGE_FLAG_ARRAY_POINTER = 0x8093;
export const GL_V2F = 0x2a20;
export const GL_V3F = 0x2a21;
export const GL_C4UB_V2F = 0x2a22;
export const GL_C4UB_V3F = 0x2a23;
export const GL_C3F_V3F = 0x2a24;
export const GL_N3F_V3F = 0x2a25;
export const GL_C4F_N3F_V3F = 0x2a26;
export const GL_T2F_V3F = 0x2a27;
export const GL_T4F_V4F = 0x2a28;
export const GL_T2F_C4UB_V3F = 0x2a29;
export const GL_T2F_C3F_V3F = 0x2a2a;
export const GL_T2F_N3F_V3F = 0x2a2b;
export const GL_T2F_C4F_N3F_V3F = 0x2a2c;
export const GL_T4F_C4F_N3F_V4F = 0x2a2d;
export const GL_MATRIX_MODE = 0x0ba0;
export const GL_MODELVIEW = 0x1700;
export const GL_PROJECTION = 0x1701;
export const GL_TEXTURE = 0x1702;
export const GL_POINT_SMOOTH = 0x0b10;
export const GL_POINT_SIZE = 0x0b11;
export const GL_POINT_SIZE_GRANULARITY = 0x0b13;
export const GL_POINT_SIZE_RANGE = 0x0b12;
export const GL_LINE_SMOOTH = 0x0b20;
export const GL_LINE_STIPPLE = 0x0b24;
export const GL_LINE_STIPPLE_PATTERN = 0x0b25;
export const GL_LINE_STIPPLE_REPEAT = 0x0b26;
export const GL_LINE_WIDTH = 0x0b21;
export const GL_LINE_WIDTH_GRANULARITY = 0x0b23;
export const GL_LINE_WIDTH_RANGE = 0x0b22;
export const GL_POINT = 0x1b00;
export const GL_LINE = 0x1b01;
export const GL_FILL = 0x1b02;
export const GL_CW = 0x0900;
export const GL_CCW = 0x0901;
export const GL_FRONT = 0x0404;
export const GL_BACK = 0x0405;
export const GL_POLYGON_MODE = 0x0b40;
export const GL_POLYGON_SMOOTH = 0x0b41;
export const GL_POLYGON_STIPPLE = 0x0b42;
export const GL_EDGE_FLAG = 0x0b43;
export const GL_CULL_FACE = 0x0b44;
export const GL_CULL_FACE_MODE = 0x0b45;
export const GL_FRONT_FACE = 0x0b46;
export const GL_POLYGON_OFFSET_FACTOR = 0x8038;
export const GL_POLYGON_OFFSET_UNITS = 0x2a00;
export const GL_POLYGON_OFFSET_POINT = 0x2a01;
export const GL_POLYGON_OFFSET_LINE = 0x2a02;
export const GL_POLYGON_OFFSET_FILL = 0x8037;
export const GL_COMPILE = 0x1300;
export const GL_COMPILE_AND_EXECUTE = 0x1301;
export const GL_LIST_BASE = 0x0b32;
export const GL_LIST_INDEX = 0x0b33;
export const GL_LIST_MODE = 0x0b30;
export const GL_NEVER = 0x0200;
export const GL_LESS = 0x0201;
export const GL_EQUAL = 0x0202;
export const GL_LEQUAL = 0x0203;
export const GL_GREATER = 0x0204;
export const GL_NOTEQUAL = 0x0205;
export const GL_GEQUAL = 0x0206;
export const GL_ALWAYS = 0x0207;
export const GL_DEPTH_TEST = 0x0b71;
export const GL_DEPTH_BITS = 0x0d56;
export const GL_DEPTH_CLEAR_VALUE = 0x0b73;
export const GL_DEPTH_FUNC = 0x0b74;
export const GL_DEPTH_RANGE = 0x0b70;
export const GL_DEPTH_WRITEMASK = 0x0b72;
export const GL_DEPTH_COMPONENT = 0x1902;
export const GL_LIGHTING = 0x0b50;
export const GL_LIGHT0 = 0x4000;
export const GL_LIGHT1 = 0x4001;
export const GL_LIGHT2 = 0x4002;
export const GL_LIGHT3 = 0x4003;
export const GL_LIGHT4 = 0x4004;
export const GL_LIGHT5 = 0x4005;
export const GL_LIGHT6 = 0x4006;
export const GL_LIGHT7 = 0x4007;
export const GL_SPOT_EXPONENT = 0x1205;
export const GL_SPOT_CUTOFF = 0x1206;
export const GL_CONSTANT_ATTENUATION = 0x1207;
export const GL_LINEAR_ATTENUATION = 0x1208;
export const GL_QUADRATIC_ATTENUATION = 0x1209;
export const GL_AMBIENT = 0x1200;
export const GL_DIFFUSE = 0x1201;
export const GL_SPECULAR = 0x1202;
export const GL_SHININESS = 0x1601;
export const GL_EMISSION = 0x1600;
export const GL_POSITION = 0x1203;
export const GL_SPOT_DIRECTION = 0x1204;
export const GL_AMBIENT_AND_DIFFUSE = 0x1602;
export const GL_COLOR_INDEXES = 0x1603;
export const GL_LIGHT_MODEL_TWO_SIDE = 0x0b52;
export const GL_LIGHT_MODEL_LOCAL_VIEWER = 0x0b51;
export const GL_LIGHT_MODEL_AMBIENT = 0x0b53;
export const GL_FRONT_AND_BACK = 0x0408;
export const GL_SHADE_MODEL = 0x0b54;
export const GL_FLAT = 0x1d00;
export const GL_SMOOTH = 0x1d01;
export const GL_COLOR_MATERIAL = 0x0b57;
export const GL_COLOR_MATERIAL_FACE = 0x0b55;
export const GL_COLOR_MATERIAL_PARAMETER = 0x0b56;
export const GL_NORMALIZE = 0x0ba1;
export const GL_CLIP_PLANE0 = 0x3000;
export const GL_CLIP_PLANE1 = 0x3001;
export const GL_CLIP_PLANE2 = 0x3002;
export const GL_CLIP_PLANE3 = 0x3003;
export const GL_CLIP_PLANE4 = 0x3004;
export const GL_CLIP_PLANE5 = 0x3005;
export const GL_ACCUM_RED_BITS = 0x0d58;
export const GL_ACCUM_GREEN_BITS = 0x0d59;
export const GL_ACCUM_BLUE_BITS = 0x0d5a;
export const GL_ACCUM_ALPHA_BITS = 0x0d5b;
export const GL_ACCUM_CLEAR_VALUE = 0x0b80;
export const GL_ACCUM = 0x0100;
export const GL_ADD = 0x0104;
export const GL_LOAD = 0x0101;
export const GL_MULT = 0x0103;
export const GL_RETURN = 0x0102;
export const GL_ALPHA_TEST = 0x0bc0;
export const GL_ALPHA_TEST_REF = 0x0bc2;
export const GL_ALPHA_TEST_FUNC = 0x0bc1;
export const GL_BLEND = 0x0be2;
export const GL_BLEND_SRC = 0x0be1;
export const GL_BLEND_DST = 0x0be0;
export const GL_ZERO = 0;
export const GL_ONE = 1;
export const GL_SRC_COLOR = 0x0300;
export const GL_ONE_MINUS_SRC_COLOR = 0x0301;
export const GL_SRC_ALPHA = 0x0302;
export const GL_ONE_MINUS_SRC_ALPHA = 0x0303;
export const GL_DST_ALPHA = 0x0304;
export const GL_ONE_MINUS_DST_ALPHA = 0x0305;
export const GL_DST_COLOR = 0x0306;
export const GL_ONE_MINUS_DST_COLOR = 0x0307;
export const GL_SRC_ALPHA_SATURATE = 0x0308;
export const GL_FEEDBACK = 0x1c01;
export const GL_RENDER = 0x1c00;
export const GL_SELECT = 0x1c02;
export const GL_2D = 0x0600;
export const GL_3D = 0x0601;
export const GL_3D_COLOR = 0x0602;
export const GL_3D_COLOR_TEXTURE = 0x0603;
export const GL_4D_COLOR_TEXTURE = 0x0604;
export const GL_POINT_TOKEN = 0x0701;
export const GL_LINE_TOKEN = 0x0702;
export const GL_LINE_RESET_TOKEN = 0x0707;
export const GL_POLYGON_TOKEN = 0x0703;
export const GL_BITMAP_TOKEN = 0x0704;
export const GL_DRAW_PIXEL_TOKEN = 0x0705;
export const GL_COPY_PIXEL_TOKEN = 0x0706;
export const GL_PASS_THROUGH_TOKEN = 0x0700;
export const GL_FEEDBACK_BUFFER_POINTER = 0x0df0;
export const GL_FEEDBACK_BUFFER_SIZE = 0x0df1;
export const GL_FEEDBACK_BUFFER_TYPE = 0x0df2;
export const GL_SELECTION_BUFFER_POINTER = 0x0df3;
export const GL_SELECTION_BUFFER_SIZE = 0x0df4;
export const GL_FOG = 0x0b60;
export const GL_FOG_MODE = 0x0b65;
export const GL_FOG_DENSITY = 0x0b62;
export const GL_FOG_COLOR = 0x0b66;
export const GL_FOG_INDEX = 0x0b61;
export const GL_FOG_START = 0x0b63;
export const GL_FOG_END = 0x0b64;
export const GL_LINEAR = 0x2601;
export const GL_EXP = 0x0800;
export const GL_EXP2 = 0x0801;
export const GL_LOGIC_OP = 0x0bf1;
export const GL_INDEX_LOGIC_OP = 0x0bf1;
export const GL_COLOR_LOGIC_OP = 0x0bf2;
export const GL_LOGIC_OP_MODE = 0x0bf0;
export const GL_CLEAR = 0x1500;
export const GL_SET = 0x150f;
export const GL_COPY = 0x1503;
export const GL_COPY_INVERTED = 0x150c;
export const GL_NOOP = 0x1505;
export const GL_INVERT = 0x150a;
export const GL_AND = 0x1501;
export const GL_NAND = 0x150e;
export const GL_OR = 0x1507;
export const GL_NOR = 0x1508;
export const GL_XOR = 0x1506;
export const GL_EQUIV = 0x1509;
export const GL_AND_REVERSE = 0x1502;
export const GL_AND_INVERTED = 0x1504;
export const GL_OR_REVERSE = 0x150b;
export const GL_OR_INVERTED = 0x150d;
export const GL_STENCIL_BITS = 0x0d57;
export const GL_STENCIL_TEST = 0x0b90;
export const GL_STENCIL_CLEAR_VALUE = 0x0b91;
export const GL_STENCIL_FUNC = 0x0b92;
export const GL_STENCIL_VALUE_MASK = 0x0b93;
export const GL_STENCIL_FAIL = 0x0b94;
export const GL_STENCIL_PASS_DEPTH_FAIL = 0x0b95;
export const GL_STENCIL_PASS_DEPTH_PASS = 0x0b96;
export const GL_STENCIL_REF = 0x0b97;
export const GL_STENCIL_WRITEMASK = 0x0b98;
export const GL_STENCIL_INDEX = 0x1901;
export const GL_KEEP = 0x1e00;
export const GL_REPLACE = 0x1e01;
export const GL_INCR = 0x1e02;
export const GL_DECR = 0x1e03;
export const GL_NONE = 0;
export const GL_LEFT = 0x0406;
export const GL_RIGHT = 0x0407;
export const GL_FRONT_LEFT = 0x0400;
export const GL_FRONT_RIGHT = 0x0401;
export const GL_BACK_LEFT = 0x0402;
export const GL_BACK_RIGHT = 0x0403;
export const GL_AUX0 = 0x0409;
export const GL_AUX1 = 0x040a;
export const GL_AUX2 = 0x040b;
export const GL_AUX3 = 0x040c;
export const GL_COLOR_INDEX = 0x1900;
export const GL_RED = 0x1903;
export const GL_GREEN = 0x1904;
export const GL_BLUE = 0x1905;
export const GL_ALPHA = 0x1906;
export const GL_LUMINANCE = 0x1909;
export const GL_LUMINANCE_ALPHA = 0x190a;
export const GL_ALPHA_BITS = 0x0d55;
export const GL_RED_BITS = 0x0d52;
export const GL_GREEN_BITS = 0x0d53;
export const GL_BLUE_BITS = 0x0d54;
export const GL_INDEX_BITS = 0x0d51;
export const GL_SUBPIXEL_BITS = 0x0d50;
export const GL_AUX_BUFFERS = 0x0c00;
export const GL_READ_BUFFER = 0x0c02;
export const GL_DRAW_BUFFER = 0x0c01;
export const GL_DOUBLEBUFFER = 0x0c32;
export const GL_STEREO = 0x0c33;
export const GL_BITMAP = 0x1a00;
export const GL_COLOR = 0x1800;
export const GL_DEPTH = 0x1801;
export const GL_STENCIL = 0x1802;
export const GL_DITHER = 0x0bd0;
export const GL_RGB = 0x1907;
export const GL_RGBA = 0x1908;
export const GL_MAX_LIST_NESTING = 0x0b31;
export const GL_MAX_EVAL_ORDER = 0x0d30;
export const GL_MAX_LIGHTS = 0x0d31;
export const GL_MAX_CLIP_PLANES = 0x0d32;
export const GL_MAX_TEXTURE_SIZE = 0x0d33;
export const GL_MAX_PIXEL_MAP_TABLE = 0x0d34;
export const GL_MAX_ATTRIB_STACK_DEPTH = 0x0d35;
export const GL_MAX_MODELVIEW_STACK_DEPTH = 0x0d36;
export const GL_MAX_NAME_STACK_DEPTH = 0x0d37;
export const GL_MAX_PROJECTION_STACK_DEPTH = 0x0d38;
export const GL_MAX_TEXTURE_STACK_DEPTH = 0x0d39;
export const GL_MAX_VIEWPORT_DIMS = 0x0d3a;
export const GL_MAX_CLIENT_ATTRIB_STACK_DEPTH = 0x0d3b;
export const GL_ATTRIB_STACK_DEPTH = 0x0bb0;
export const GL_CLIENT_ATTRIB_STACK_DEPTH = 0x0bb1;
export const GL_COLOR_CLEAR_VALUE = 0x0c22;
export const GL_COLOR_WRITEMASK = 0x0c23;
export const GL_CURRENT_INDEX = 0x0b01;
export const GL_CURRENT_COLOR = 0x0b00;
export const GL_CURRENT_NORMAL = 0x0b02;
export const GL_CURRENT_RASTER_COLOR = 0x0b04;
export const GL_CURRENT_RASTER_DISTANCE = 0x0b09;
export const GL_CURRENT_RASTER_INDEX = 0x0b05;
export const GL_CURRENT_RASTER_POSITION = 0x0b07;
export const GL_CURRENT_RASTER_TEXTURE_COORDS = 0x0b06;
export const GL_CURRENT_RASTER_POSITION_VALID = 0x0b08;
export const GL_CURRENT_TEXTURE_COORDS = 0x0b03;
export const GL_INDEX_CLEAR_VALUE = 0x0c20;
export const GL_INDEX_MODE = 0x0c30;
export const GL_INDEX_WRITEMASK = 0x0c21;
export const GL_MODELVIEW_MATRIX = 0x0ba6;
export const GL_MODELVIEW_STACK_DEPTH = 0x0ba3;
export const GL_NAME_STACK_DEPTH = 0x0d70;
export const GL_PROJECTION_MATRIX = 0x0ba7;
export const GL_PROJECTION_STACK_DEPTH = 0x0ba4;
export const GL_RENDER_MODE = 0x0c40;
export const GL_RGBA_MODE = 0x0c31;
export const GL_TEXTURE_MATRIX = 0x0ba8;
export const GL_TEXTURE_STACK_DEPTH = 0x0ba5;
export const GL_VIEWPORT = 0x0ba2;
export const GL_AUTO_NORMAL = 0x0d80;
export const GL_MAP1_COLOR_4 = 0x0d90;
export const GL_MAP1_INDEX = 0x0d91;
export const GL_MAP1_NORMAL = 0x0d92;
export const GL_MAP1_TEXTURE_COORD_1 = 0x0d93;
export const GL_MAP1_TEXTURE_COORD_2 = 0x0d94;
export const GL_MAP1_TEXTURE_COORD_3 = 0x0d95;
export const GL_MAP1_TEXTURE_COORD_4 = 0x0d96;
export const GL_MAP1_VERTEX_3 = 0x0d97;
export const GL_MAP1_VERTEX_4 = 0x0d98;
export const GL_MAP2_COLOR_4 = 0x0db0;
export const GL_MAP2_INDEX = 0x0db1;
export const GL_MAP2_NORMAL = 0x0db2;
export const GL_MAP2_TEXTURE_COORD_1 = 0x0db3;
export const GL_MAP2_TEXTURE_COORD_2 = 0x0db4;
export const GL_MAP2_TEXTURE_COORD_3 = 0x0db5;
export const GL_MAP2_TEXTURE_COORD_4 = 0x0db6;
export const GL_MAP2_VERTEX_3 = 0x0db7;
export const GL_MAP2_VERTEX_4 = 0x0db8;
export const GL_MAP1_GRID_DOMAIN = 0x0dd0;
export const GL_MAP1_GRID_SEGMENTS = 0x0dd1;
export const GL_MAP2_GRID_DOMAIN = 0x0dd2;
export const GL_MAP2_GRID_SEGMENTS = 0x0dd3;
export const GL_COEFF = 0x0a00;
export const GL_ORDER = 0x0a01;
export const GL_DOMAIN = 0x0a02;
export const GL_PERSPECTIVE_CORRECTION_HINT = 0x0c50;
export const GL_POINT_SMOOTH_HINT = 0x0c51;
export const GL_LINE_SMOOTH_HINT = 0x0c52;
export const GL_POLYGON_SMOOTH_HINT = 0x0c53;
export const GL_FOG_HINT = 0x0c54;
export const GL_DONT_CARE = 0x1100;
export const GL_FASTEST = 0x1101;
export const GL_NICEST = 0x1102;
export const GL_SCISSOR_BOX = 0x0c10;
export const GL_SCISSOR_TEST = 0x0c11;
export const GL_MAP_COLOR = 0x0d10;
export const GL_MAP_STENCIL = 0x0d11;
export const GL_INDEX_SHIFT = 0x0d12;
export const GL_INDEX_OFFSET = 0x0d13;
export const GL_RED_SCALE = 0x0d14;
export const GL_RED_BIAS = 0x0d15;
export const GL_GREEN_SCALE = 0x0d18;
export const GL_GREEN_BIAS = 0x0d19;
export const GL_BLUE_SCALE = 0x0d1a;
export const GL_BLUE_BIAS = 0x0d1b;
export const GL_ALPHA_SCALE = 0x0d1c;
export const GL_ALPHA_BIAS = 0x0d1d;
export const GL_DEPTH_SCALE = 0x0d1e;
export const GL_DEPTH_BIAS = 0x0d1f;
export const GL_PIXEL_MAP_S_TO_S_SIZE = 0x0cb1;
export const GL_PIXEL_MAP_I_TO_I_SIZE = 0x0cb0;
export const GL_PIXEL_MAP_I_TO_R_SIZE = 0x0cb2;
export const GL_PIXEL_MAP_I_TO_G_SIZE = 0x0cb3;
export const GL_PIXEL_MAP_I_TO_B_SIZE = 0x0cb4;
export const GL_PIXEL_MAP_I_TO_A_SIZE = 0x0cb5;
export const GL_PIXEL_MAP_R_TO_R_SIZE = 0x0cb6;
export const GL_PIXEL_MAP_G_TO_G_SIZE = 0x0cb7;
export const GL_PIXEL_MAP_B_TO_B_SIZE = 0x0cb8;
export const GL_PIXEL_MAP_A_TO_A_SIZE = 0x0cb9;
export const GL_PIXEL_MAP_S_TO_S = 0x0c71;
export const GL_PIXEL_MAP_I_TO_I = 0x0c70;
export const GL_PIXEL_MAP_I_TO_R = 0x0c72;
export const GL_PIXEL_MAP_I_TO_G = 0x0c73;
export const GL_PIXEL_MAP_I_TO_B = 0x0c74;
export const GL_PIXEL_MAP_I_TO_A = 0x0c75;
export const GL_PIXEL_MAP_R_TO_R = 0x0c76;
export const GL_PIXEL_MAP_G_TO_G = 0x0c77;
export const GL_PIXEL_MAP_B_TO_B = 0x0c78;
export const GL_PIXEL_MAP_A_TO_A = 0x0c79;
export const GL_PACK_ALIGNMENT = 0x0d05;
export const GL_PACK_LSB_FIRST = 0x0d01;
export const GL_PACK_ROW_LENGTH = 0x0d02;
export const GL_PACK_SKIP_PIXELS = 0x0d04;
export const GL_PACK_SKIP_ROWS = 0x0d03;
export const GL_PACK_SWAP_BYTES = 0x0d00;
export const GL_UNPACK_ALIGNMENT = 0x0cf5;
export const GL_UNPACK_LSB_FIRST = 0x0cf1;
export const GL_UNPACK_ROW_LENGTH = 0x0cf2;
export const GL_UNPACK_SKIP_PIXELS = 0x0cf4;
export const GL_UNPACK_SKIP_ROWS = 0x0cf3;
export const GL_UNPACK_SWAP_BYTES = 0x0cf0;
export const GL_ZOOM_X = 0x0d16;
export const GL_ZOOM_Y = 0x0d17;
export const GL_TEXTURE_ENV = 0x2300;
export const GL_TEXTURE_ENV_MODE = 0x2200;
export const GL_TEXTURE_1D = 0x0de0;
export const GL_TEXTURE_2D = 0x0de1;
export const GL_TEXTURE_WRAP_S = 0x2802;
export const GL_TEXTURE_WRAP_T = 0x2803;
export const GL_TEXTURE_MAG_FILTER = 0x2800;
export const GL_TEXTURE_MIN_FILTER = 0x2801;
export const GL_TEXTURE_ENV_COLOR = 0x2201;
export const GL_TEXTURE_GEN_S = 0x0c60;
export const GL_TEXTURE_GEN_T = 0x0c61;
export const GL_TEXTURE_GEN_R = 0x0c62;
export const GL_TEXTURE_GEN_Q = 0x0c63;
export const GL_TEXTURE_GEN_MODE = 0x2500;
export const GL_TEXTURE_BORDER_COLOR = 0x1004;
export const GL_TEXTURE_WIDTH = 0x1000;
export const GL_TEXTURE_HEIGHT = 0x1001;
export const GL_TEXTURE_BORDER = 0x1005;
export const GL_TEXTURE_COMPONENTS = 0x1003;
export const GL_TEXTURE_RED_SIZE = 0x805c;
export const GL_TEXTURE_GREEN_SIZE = 0x805d;
export const GL_TEXTURE_BLUE_SIZE = 0x805e;
export const GL_TEXTURE_ALPHA_SIZE = 0x805f;
export const GL_TEXTURE_LUMINANCE_SIZE = 0x8060;
export const GL_TEXTURE_INTENSITY_SIZE = 0x8061;
export const GL_NEAREST_MIPMAP_NEAREST = 0x2700;
export const GL_NEAREST_MIPMAP_LINEAR = 0x2702;
export const GL_LINEAR_MIPMAP_NEAREST = 0x2701;
export const GL_LINEAR_MIPMAP_LINEAR = 0x2703;
export const GL_OBJECT_LINEAR = 0x2401;
export const GL_OBJECT_PLANE = 0x2501;
export const GL_EYE_LINEAR = 0x2400;
export const GL_EYE_PLANE = 0x2502;
export const GL_SPHERE_MAP = 0x2402;
export const GL_DECAL = 0x2101;
export const GL_MODULATE = 0x2100;
export const GL_NEAREST = 0x2600;
export const GL_REPEAT = 0x2901;
export const GL_CLAMP = 0x2900;
export const GL_S = 0x2000;
export const GL_T = 0x2001;
export const GL_R = 0x2002;
export const GL_Q = 0x2003;
export const GL_VENDOR = 0x1f00;
export const GL_RENDERER = 0x1f01;
export const GL_VERSION = 0x1f02;
export const GL_EXTENSIONS = 0x1f03;
export const GL_NO_ERROR = 0;
export const GL_INVALID_ENUM = 0x0500;
export const GL_INVALID_VALUE = 0x0501;
export const GL_INVALID_OPERATION = 0x0502;
export const GL_STACK_OVERFLOW = 0x0503;
export const GL_STACK_UNDERFLOW = 0x0504;
export const GL_OUT_OF_MEMORY = 0x0505;
export const GL_CURRENT_BIT = 0x00000001;
export const GL_POINT_BIT = 0x00000002;
export const GL_LINE_BIT = 0x00000004;
export const GL_POLYGON_BIT = 0x00000008;
export const GL_POLYGON_STIPPLE_BIT = 0x00000010;
export const GL_PIXEL_MODE_BIT = 0x00000020;
export const GL_LIGHTING_BIT = 0x00000040;
export const GL_FOG_BIT = 0x00000080;
export const GL_DEPTH_BUFFER_BIT = 0x00000100;
export const GL_ACCUM_BUFFER_BIT = 0x00000200;
export const GL_STENCIL_BUFFER_BIT = 0x00000400;
export const GL_VIEWPORT_BIT = 0x00000800;
export const GL_TRANSFORM_BIT = 0x00001000;
export const GL_ENABLE_BIT = 0x00002000;
export const GL_COLOR_BUFFER_BIT = 0x00004000;
export const GL_HINT_BIT = 0x00008000;
export const GL_EVAL_BIT = 0x00010000;
export const GL_LIST_BIT = 0x00020000;
export const GL_TEXTURE_BIT = 0x00040000;
export const GL_SCISSOR_BIT = 0x00080000;
export const GL_ALL_ATTRIB_BITS = 0xffffffff;
export const GL_PROXY_TEXTURE_1D = 0x8063;
export const GL_PROXY_TEXTURE_2D = 0x8064;
export const GL_TEXTURE_PRIORITY = 0x8066;
export const GL_TEXTURE_RESIDENT = 0x8067;
export const GL_TEXTURE_BINDING_1D = 0x8068;
export const GL_TEXTURE_BINDING_2D = 0x8069;
export const GL_TEXTURE_INTERNAL_FORMAT = 0x1003;
export const GL_ALPHA4 = 0x803b;
export const GL_ALPHA8 = 0x803c;
export const GL_ALPHA12 = 0x803d;
export const GL_ALPHA16 = 0x803e;
export const GL_LUMINANCE4 = 0x803f;
export const GL_LUMINANCE8 = 0x8040;
export const GL_LUMINANCE12 = 0x8041;
export const GL_LUMINANCE16 = 0x8042;
export const GL_LUMINANCE4_ALPHA4 = 0x8043;
export const GL_LUMINANCE6_ALPHA2 = 0x8044;
export const GL_LUMINANCE8_ALPHA8 = 0x8045;
export const GL_LUMINANCE12_ALPHA4 = 0x8046;
export const GL_LUMINANCE12_ALPHA12 = 0x8047;
export const GL_LUMINANCE16_ALPHA16 = 0x8048;
export const GL_INTENSITY = 0x8049;
export const GL_INTENSITY4 = 0x804a;
export const GL_INTENSITY8 = 0x804b;
export const GL_INTENSITY12 = 0x804c;
export const GL_INTENSITY16 = 0x804d;
export const GL_R3_G3_B2 = 0x2a10;
export const GL_RGB4 = 0x804f;
export const GL_RGB5 = 0x8050;
export const GL_RGB8 = 0x8051;
export const GL_RGB10 = 0x8052;
export const GL_RGB12 = 0x8053;
export const GL_RGB16 = 0x8054;
export const GL_RGBA2 = 0x8055;
export const GL_RGBA4 = 0x8056;
export const GL_RGB5_A1 = 0x8057;
export const GL_RGBA8 = 0x8058;
export const GL_RGB10_A2 = 0x8059;
export const GL_RGBA12 = 0x805a;
export const GL_RGBA16 = 0x805b;
export const GL_CLIENT_PIXEL_STORE_BIT = 0x00000001;
export const GL_CLIENT_VERTEX_ARRAY_BIT = 0x00000002;
export const GL_ALL_CLIENT_ATTRIB_BITS = 0xffffffff;
export const GL_CLIENT_ALL_ATTRIB_BITS = 0xffffffff;
export const GL_RESCALE_NORMAL = 0x803a;
export const GL_CLAMP_TO_EDGE = 0x812f;
export const GL_MAX_ELEMENTS_VERTICES = 0x80e8;
export const GL_MAX_ELEMENTS_INDICES = 0x80e9;
export const GL_BGR = 0x80e0;
export const GL_BGRA = 0x80e1;
export const GL_UNSIGNED_BYTE_3_3_2 = 0x8032;
export const GL_UNSIGNED_BYTE_2_3_3_REV = 0x8362;
export const GL_UNSIGNED_SHORT_5_6_5 = 0x8363;
export const GL_UNSIGNED_SHORT_5_6_5_REV = 0x8364;
export const GL_UNSIGNED_SHORT_4_4_4_4 = 0x8033;
export const GL_UNSIGNED_SHORT_4_4_4_4_REV = 0x8365;
export const GL_UNSIGNED_SHORT_5_5_5_1 = 0x8034;
export const GL_UNSIGNED_SHORT_1_5_5_5_REV = 0x8366;
export const GL_UNSIGNED_INT_8_8_8_8 = 0x8035;
export const GL_UNSIGNED_INT_8_8_8_8_REV = 0x8367;
export const GL_UNSIGNED_INT_10_10_10_2 = 0x8036;
export const GL_UNSIGNED_INT_2_10_10_10_REV = 0x8368;
export const GL_LIGHT_MODEL_COLOR_CONTROL = 0x81f8;
export const GL_SINGLE_COLOR = 0x81f9;
export const GL_SEPARATE_SPECULAR_COLOR = 0x81fa;
export const GL_TEXTURE_MIN_LOD = 0x813a;
export const GL_TEXTURE_MAX_LOD = 0x813b;
export const GL_TEXTURE_BASE_LEVEL = 0x813c;
export const GL_TEXTURE_MAX_LEVEL = 0x813d;
export const GL_SMOOTH_POINT_SIZE_RANGE = 0x0b12;
export const GL_SMOOTH_POINT_SIZE_GRANULARITY = 0x0b13;
export const GL_SMOOTH_LINE_WIDTH_RANGE = 0x0b22;
export const GL_SMOOTH_LINE_WIDTH_GRANULARITY = 0x0b23;
export const GL_ALIASED_POINT_SIZE_RANGE = 0x846d;
export const GL_ALIASED_LINE_WIDTH_RANGE = 0x846e;
export const GL_PACK_SKIP_IMAGES = 0x806b;
export const GL_PACK_IMAGE_HEIGHT = 0x806c;
export const GL_UNPACK_SKIP_IMAGES = 0x806d;
export const GL_UNPACK_IMAGE_HEIGHT = 0x806e;
export const GL_TEXTURE_3D = 0x806f;
export const GL_PROXY_TEXTURE_3D = 0x8070;
export const GL_TEXTURE_DEPTH = 0x8071;
export const GL_TEXTURE_WRAP_R = 0x8072;
export const GL_MAX_3D_TEXTURE_SIZE = 0x8073;
export const GL_TEXTURE_BINDING_3D = 0x806a;
export const GL_CONSTANT_COLOR = 0x8001;
export const GL_ONE_MINUS_CONSTANT_COLOR = 0x8002;
export const GL_CONSTANT_ALPHA = 0x8003;
export const GL_ONE_MINUS_CONSTANT_ALPHA = 0x8004;
export const GL_COLOR_TABLE = 0x80d0;
export const GL_POST_CONVOLUTION_COLOR_TABLE = 0x80d1;
export const GL_POST_COLOR_MATRIX_COLOR_TABLE = 0x80d2;
export const GL_PROXY_COLOR_TABLE = 0x80d3;
export const GL_PROXY_POST_CONVOLUTION_COLOR_TABLE = 0x80d4;
export const GL_PROXY_POST_COLOR_MATRIX_COLOR_TABLE = 0x80d5;
export const GL_COLOR_TABLE_SCALE = 0x80d6;
export const GL_COLOR_TABLE_BIAS = 0x80d7;
export const GL_COLOR_TABLE_FORMAT = 0x80d8;
export const GL_COLOR_TABLE_WIDTH = 0x80d9;
export const GL_COLOR_TABLE_RED_SIZE = 0x80da;
export const GL_COLOR_TABLE_GREEN_SIZE = 0x80db;
export const GL_COLOR_TABLE_BLUE_SIZE = 0x80dc;
export const GL_COLOR_TABLE_ALPHA_SIZE = 0x80dd;
export const GL_COLOR_TABLE_LUMINANCE_SIZE = 0x80de;
export const GL_COLOR_TABLE_INTENSITY_SIZE = 0x80df;
export const GL_CONVOLUTION_1D = 0x8010;
export const GL_CONVOLUTION_2D = 0x8011;
export const GL_SEPARABLE_2D = 0x8012;
export const GL_CONVOLUTION_BORDER_MODE = 0x8013;
export const GL_CONVOLUTION_FILTER_SCALE = 0x8014;
export const GL_CONVOLUTION_FILTER_BIAS = 0x8015;
export const GL_REDUCE = 0x8016;
export const GL_CONVOLUTION_FORMAT = 0x8017;
export const GL_CONVOLUTION_WIDTH = 0x8018;
export const GL_CONVOLUTION_HEIGHT = 0x8019;
export const GL_MAX_CONVOLUTION_WIDTH = 0x801a;
export const GL_MAX_CONVOLUTION_HEIGHT = 0x801b;
export const GL_POST_CONVOLUTION_RED_SCALE = 0x801c;
export const GL_POST_CONVOLUTION_GREEN_SCALE = 0x801d;
export const GL_POST_CONVOLUTION_BLUE_SCALE = 0x801e;
export const GL_POST_CONVOLUTION_ALPHA_SCALE = 0x801f;
export const GL_POST_CONVOLUTION_RED_BIAS = 0x8020;
export const GL_POST_CONVOLUTION_GREEN_BIAS = 0x8021;
export const GL_POST_CONVOLUTION_BLUE_BIAS = 0x8022;
export const GL_POST_CONVOLUTION_ALPHA_BIAS = 0x8023;
export const GL_CONSTANT_BORDER = 0x8151;
export const GL_REPLICATE_BORDER = 0x8153;
export const GL_CONVOLUTION_BORDER_COLOR = 0x8154;
export const GL_COLOR_MATRIX = 0x80b1;
export const GL_COLOR_MATRIX_STACK_DEPTH = 0x80b2;
export const GL_MAX_COLOR_MATRIX_STACK_DEPTH = 0x80b3;
export const GL_POST_COLOR_MATRIX_RED_SCALE = 0x80b4;
export const GL_POST_COLOR_MATRIX_GREEN_SCALE = 0x80b5;
export const GL_POST_COLOR_MATRIX_BLUE_SCALE = 0x80b6;
export const GL_POST_COLOR_MATRIX_ALPHA_SCALE = 0x80b7;
export const GL_POST_COLOR_MATRIX_RED_BIAS = 0x80b8;
export const GL_POST_COLOR_MATRIX_GREEN_BIAS = 0x80b9;
export const GL_POST_COLOR_MATRIX_BLUE_BIAS = 0x80ba;
export const GL_POST_COLOR_MATRIX_ALPHA_BIAS = 0x80bb;
export const GL_HISTOGRAM = 0x8024;
export const GL_PROXY_HISTOGRAM = 0x8025;
export const GL_HISTOGRAM_WIDTH = 0x8026;
export const GL_HISTOGRAM_FORMAT = 0x8027;
export const GL_HISTOGRAM_RED_SIZE = 0x8028;
export const GL_HISTOGRAM_GREEN_SIZE = 0x8029;
export const GL_HISTOGRAM_BLUE_SIZE = 0x802a;
export const GL_HISTOGRAM_ALPHA_SIZE = 0x802b;
export const GL_HISTOGRAM_LUMINANCE_SIZE = 0x802c;
export const GL_HISTOGRAM_SINK = 0x802d;
export const GL_MINMAX = 0x802e;
export const GL_MINMAX_FORMAT = 0x802f;
export const GL_MINMAX_SINK = 0x8030;
export const GL_TABLE_TOO_LARGE = 0x8031;
export const GL_BLEND_EQUATION = 0x8009;
export const GL_MIN = 0x8007;
export const GL_MAX = 0x8008;
export const GL_FUNC_ADD = 0x8006;
export const GL_FUNC_SUBTRACT = 0x800a;
export const GL_FUNC_REVERSE_SUBTRACT = 0x800b;
export const GL_BLEND_COLOR = 0x8005;
export const GL_TEXTURE0 = 0x84c0;
export const GL_TEXTURE1 = 0x84c1;
export const GL_TEXTURE2 = 0x84c2;
export const GL_TEXTURE3 = 0x84c3;
export const GL_TEXTURE4 = 0x84c4;
export const GL_TEXTURE5 = 0x84c5;
export const GL_TEXTURE6 = 0x84c6;
export const GL_TEXTURE7 = 0x84c7;
export const GL_TEXTURE8 = 0x84c8;
export const GL_TEXTURE9 = 0x84c9;
export const GL_TEXTURE10 = 0x84ca;
export const GL_TEXTURE11 = 0x84cb;
export const GL_TEXTURE12 = 0x84cc;
export const GL_TEXTURE13 = 0x84cd;
export const GL_TEXTURE14 = 0x84ce;
export const GL_TEXTURE15 = 0x84cf;
export const GL_TEXTURE16 = 0x84d0;
export const GL_TEXTURE17 = 0x84d1;
export const GL_TEXTURE18 = 0x84d2;
export const GL_TEXTURE19 = 0x84d3;
export const GL_TEXTURE20 = 0x84d4;
export const GL_TEXTURE21 = 0x84d5;
export const GL_TEXTURE22 = 0x84d6;
export const GL_TEXTURE23 = 0x84d7;
export const GL_TEXTURE24 = 0x84d8;
export const GL_TEXTURE25 = 0x84d9;
export const GL_TEXTURE26 = 0x84da;
export const GL_TEXTURE27 = 0x84db;
export const GL_TEXTURE28 = 0x84dc;
export const GL_TEXTURE29 = 0x84dd;
export const GL_TEXTURE30 = 0x84de;
export const GL_TEXTURE31 = 0x84df;
export const GL_ACTIVE_TEXTURE = 0x84e0;
export const GL_CLIENT_ACTIVE_TEXTURE = 0x84e1;
export const GL_MAX_TEXTURE_UNITS = 0x84e2;
export const GL_NORMAL_MAP = 0x8511;
export const GL_REFLECTION_MAP = 0x8512;
export const GL_TEXTURE_CUBE_MAP = 0x8513;
export const GL_TEXTURE_BINDING_CUBE_MAP = 0x8514;
export const GL_TEXTURE_CUBE_MAP_POSITIVE_X = 0x8515;
export const GL_TEXTURE_CUBE_MAP_NEGATIVE_X = 0x8516;
export const GL_TEXTURE_CUBE_MAP_POSITIVE_Y = 0x8517;
export const GL_TEXTURE_CUBE_MAP_NEGATIVE_Y = 0x8518;
export const GL_TEXTURE_CUBE_MAP_POSITIVE_Z = 0x8519;
export const GL_TEXTURE_CUBE_MAP_NEGATIVE_Z = 0x851a;
export const GL_PROXY_TEXTURE_CUBE_MAP = 0x851b;
export const GL_MAX_CUBE_MAP_TEXTURE_SIZE = 0x851c;
export const GL_COMPRESSED_ALPHA = 0x84e9;
export const GL_COMPRESSED_LUMINANCE = 0x84ea;
export const GL_COMPRESSED_LUMINANCE_ALPHA = 0x84eb;
export const GL_COMPRESSED_INTENSITY = 0x84ec;
export const GL_COMPRESSED_RGB = 0x84ed;
export const GL_COMPRESSED_RGBA = 0x84ee;
export const GL_TEXTURE_COMPRESSION_HINT = 0x84ef;
export const GL_TEXTURE_COMPRESSED_IMAGE_SIZE = 0x86a0;
export const GL_TEXTURE_COMPRESSED = 0x86a1;
export const GL_NUM_COMPRESSED_TEXTURE_FORMATS = 0x86a2;
export const GL_COMPRESSED_TEXTURE_FORMATS = 0x86a3;
export const GL_MULTISAMPLE = 0x809d;
export const GL_SAMPLE_ALPHA_TO_COVERAGE = 0x809e;
export const GL_SAMPLE_ALPHA_TO_ONE = 0x809f;
export const GL_SAMPLE_COVERAGE = 0x80a0;
export const GL_SAMPLE_BUFFERS = 0x80a8;
export const GL_SAMPLES = 0x80a9;
export const GL_SAMPLE_COVERAGE_VALUE = 0x80aa;
export const GL_SAMPLE_COVERAGE_INVERT = 0x80ab;
export const GL_MULTISAMPLE_BIT = 0x20000000;
export const GL_TRANSPOSE_MODELVIEW_MATRIX = 0x84e3;
export const GL_TRANSPOSE_PROJECTION_MATRIX = 0x84e4;
export const GL_TRANSPOSE_TEXTURE_MATRIX = 0x84e5;
export const GL_TRANSPOSE_COLOR_MATRIX = 0x84e6;
export const GL_COMBINE = 0x8570;
export const GL_COMBINE_RGB = 0x8571;
export const GL_COMBINE_ALPHA = 0x8572;
export const GL_SOURCE0_RGB = 0x8580;
export const GL_SOURCE1_RGB = 0x8581;
export const GL_SOURCE2_RGB = 0x8582;
export const GL_SOURCE0_ALPHA = 0x8588;
export const GL_SOURCE1_ALPHA = 0x8589;
export const GL_SOURCE2_ALPHA = 0x858a;
export const GL_OPERAND0_RGB = 0x8590;
export const GL_OPERAND1_RGB = 0x8591;
export const GL_OPERAND2_RGB = 0x8592;
export const GL_OPERAND0_ALPHA = 0x8598;
export const GL_OPERAND1_ALPHA = 0x8599;
export const GL_OPERAND2_ALPHA = 0x859a;
export const GL_RGB_SCALE = 0x8573;
export const GL_ADD_SIGNED = 0x8574;
export const GL_INTERPOLATE = 0x8575;
export const GL_SUBTRACT = 0x84e7;
export const GL_CONSTANT = 0x8576;
export const GL_PRIMARY_COLOR = 0x8577;
export const GL_PREVIOUS = 0x8578;
export const GL_DOT3_RGB = 0x86ae;
export const GL_DOT3_RGBA = 0x86af;
export const GL_CLAMP_TO_BORDER = 0x812d;
export const GL_ARB_multitexture = 1;
export const GL_TEXTURE0_ARB = 0x84c0;
export const GL_TEXTURE1_ARB = 0x84c1;
export const GL_TEXTURE2_ARB = 0x84c2;
export const GL_TEXTURE3_ARB = 0x84c3;
export const GL_TEXTURE4_ARB = 0x84c4;
export const GL_TEXTURE5_ARB = 0x84c5;
export const GL_TEXTURE6_ARB = 0x84c6;
export const GL_TEXTURE7_ARB = 0x84c7;
export const GL_TEXTURE8_ARB = 0x84c8;
export const GL_TEXTURE9_ARB = 0x84c9;
export const GL_TEXTURE10_ARB = 0x84ca;
export const GL_TEXTURE11_ARB = 0x84cb;
export const GL_TEXTURE12_ARB = 0x84cc;
export const GL_TEXTURE13_ARB = 0x84cd;
export const GL_TEXTURE14_ARB = 0x84ce;
export const GL_TEXTURE15_ARB = 0x84cf;
export const GL_TEXTURE16_ARB = 0x84d0;
export const GL_TEXTURE17_ARB = 0x84d1;
export const GL_TEXTURE18_ARB = 0x84d2;
export const GL_TEXTURE19_ARB = 0x84d3;
export const GL_TEXTURE20_ARB = 0x84d4;
export const GL_TEXTURE21_ARB = 0x84d5;
export const GL_TEXTURE22_ARB = 0x84d6;
export const GL_TEXTURE23_ARB = 0x84d7;
export const GL_TEXTURE24_ARB = 0x84d8;
export const GL_TEXTURE25_ARB = 0x84d9;
export const GL_TEXTURE26_ARB = 0x84da;
export const GL_TEXTURE27_ARB = 0x84db;
export const GL_TEXTURE28_ARB = 0x84dc;
export const GL_TEXTURE29_ARB = 0x84dd;
export const GL_TEXTURE30_ARB = 0x84de;
export const GL_TEXTURE31_ARB = 0x84df;
export const GL_ACTIVE_TEXTURE_ARB = 0x84e0;
export const GL_CLIENT_ACTIVE_TEXTURE_ARB = 0x84e1;
export const GL_MAX_TEXTURE_UNITS_ARB = 0x84e2;
export const GL_DEPTH_STENCIL_MESA = 0x8750;
export const GL_UNSIGNED_INT_24_8_MESA = 0x8751;
export const GL_UNSIGNED_INT_8_24_REV_MESA = 0x8752;
export const GL_UNSIGNED_SHORT_15_1_MESA = 0x8753;
export const GL_UNSIGNED_SHORT_1_15_REV_MESA = 0x8754;
export const GL_ALPHA_BLEND_EQUATION_ATI = 0x883d;
export const GL_OES_EGL_image = 1;

const libGL = dlopen('libGL.so', RTLD_NOW);

define('glAccum', dlsym(libGL, 'glAccum'), null, 'void', 'unsigned int', 'float');
export function glAccum(op, value) {
  return call('glAccum', op, value);
}

define('glAlphaFunc', dlsym(libGL, 'glAlphaFunc'), null, 'void', 'unsigned int', 'float');
export function glAlphaFunc(func, ref) {
  return call('glAlphaFunc', func, ref);
}

define('glArrayElement', dlsym(libGL, 'glArrayElement'), null, 'void', 'int');
export function glArrayElement(i) {
  return call('glArrayElement', i);
}

define('glBegin', dlsym(libGL, 'glBegin'), null, 'void', 'unsigned int');
export function glBegin(mode) {
  return call('glBegin', mode);
}

define('glEnd', dlsym(libGL, 'glEnd'), null, 'void');
export function glEnd() {
  return call('glEnd');
}

define('glFlush', dlsym(libGL, 'glFlush'), null, 'void');
export function glFlush() {
  return call('glFlush');
}

define('glLoadIdentity', dlsym(libGL, 'glLoadIdentity'), null, 'void');
export function glLoadIdentity() {
  return call('glLoadIdentity');
}

define('glPushMatrix', dlsym(libGL, 'glPushMatrix'), null, 'void');
export function glPushMatrix() {
  return call('glPushMatrix');
}

define('glPopMatrix', dlsym(libGL, 'glPopMatrix'), null, 'void');
export function glPopMatrix() {
  return call('glPopMatrix');
}

define('glBindTexture', dlsym(libGL, 'glBindTexture'), null, 'void', 'unsigned int', 'unsigned int');
export function glBindTexture(target, texture) {
  return call('glBindTexture', target, texture);
}

define('glBitmap', dlsym(libGL, 'glBitmap'), null, 'void', 'int', 'int', 'float', 'float', 'float', 'float', 'void *');
export function glBitmap(width, height, xorig, yorig, xmove, ymove, bitmap) {
  return call('glBitmap', width, height, xorig, yorig, xmove, ymove, bitmap);
}

define('glBlendFunc', dlsym(libGL, 'glBlendFunc'), null, 'void', 'unsigned int', 'unsigned int');
export function glBlendFunc(sfactor, dfactor) {
  return call('glBlendFunc', sfactor, dfactor);
}

define('glCallList', dlsym(libGL, 'glCallList'), null, 'void', 'unsigned int');
export function glCallList(list) {
  return call('glCallList', list);
}

define('glCallLists', dlsym(libGL, 'glCallLists'), null, 'void', 'int', 'unsigned int', 'void *');
export function glCallLists(n, type, lists) {
  return call('glCallLists', n, type, lists);
}

define('glClear', dlsym(libGL, 'glClear'), null, 'void', 'unsigned int');
export function glClear(mask) {
  return call('glClear', mask);
}

define('glClearAccum', dlsym(libGL, 'glClearAccum'), null, 'void', 'float', 'float', 'float', 'float');
export function glClearAccum(red, green, blue, alpha) {
  return call('glClearAccum', red, green, blue, alpha);
}

define('glClearColor', dlsym(libGL, 'glClearColor'), null, 'void', 'float', 'float', 'float', 'float');
export function glClearColor(red, green, blue, alpha) {
  return call('glClearColor', red, green, blue, alpha);
}

define('glClearDepth', dlsym(libGL, 'glClearDepth'), null, 'void', 'double');
export function glClearDepth(depth) {
  return call('glClearDepth', depth);
}

define('glClearIndex', dlsym(libGL, 'glClearIndex'), null, 'void', 'float');
export function glClearIndex(c) {
  return call('glClearIndex', c);
}

define('glClearStencil', dlsym(libGL, 'glClearStencil'), null, 'void', 'int');
export function glClearStencil(s) {
  return call('glClearStencil', s);
}

define('glClipPlane', dlsym(libGL, 'glClipPlane'), null, 'void', 'unsigned int', 'void *');
export function glClipPlane(plane, equation) {
  return call('glClipPlane', plane, equation);
}

define('glColor3b', dlsym(libGL, 'glColor3b'), null, 'void', 'sint8', 'sint8', 'sint8');
export function glColor3b(red, green, blue) {
  return call('glColor3b', red, green, blue);
}

define('glColor3bv', dlsym(libGL, 'glColor3bv'), null, 'void', 'void *');
export function glColor3bv(v) {
  return call('glColor3bv', v);
}

define('glColor3d', dlsym(libGL, 'glColor3d'), null, 'void', 'double', 'double', 'double');
export function glColor3d(red, green, blue) {
  return call('glColor3d', red, green, blue);
}

define('glColor3dv', dlsym(libGL, 'glColor3dv'), null, 'void', 'void *');
export function glColor3dv(v) {
  return call('glColor3dv', v);
}

define('glColor3f', dlsym(libGL, 'glColor3f'), null, 'void', 'float', 'float', 'float');
export function glColor3f(red, green, blue) {
  return call('glColor3f', red, green, blue);
}

define('glColor3fv', dlsym(libGL, 'glColor3fv'), null, 'void', 'void *');
export function glColor3fv(v) {
  return call('glColor3fv', v);
}

define('glColor3i', dlsym(libGL, 'glColor3i'), null, 'void', 'int', 'int', 'int');
export function glColor3i(red, green, blue) {
  return call('glColor3i', red, green, blue);
}

define('glColor3iv', dlsym(libGL, 'glColor3iv'), null, 'void', 'void *');
export function glColor3iv(v) {
  return call('glColor3iv', v);
}

define('glColor3s', dlsym(libGL, 'glColor3s'), null, 'void', 'short', 'short', 'short');
export function glColor3s(red, green, blue) {
  return call('glColor3s', red, green, blue);
}

define('glColor3sv', dlsym(libGL, 'glColor3sv'), null, 'void', 'void *');
export function glColor3sv(v) {
  return call('glColor3sv', v);
}

define('glColor3ub', dlsym(libGL, 'glColor3ub'), null, 'void', 'unsigned char', 'unsigned char', 'unsigned char');
export function glColor3ub(red, green, blue) {
  return call('glColor3ub', red, green, blue);
}

define('glColor3ubv', dlsym(libGL, 'glColor3ubv'), null, 'void', 'void *');
export function glColor3ubv(v) {
  return call('glColor3ubv', v);
}

define('glColor3ui', dlsym(libGL, 'glColor3ui'), null, 'void', 'unsigned int', 'unsigned int', 'unsigned int');
export function glColor3ui(red, green, blue) {
  return call('glColor3ui', red, green, blue);
}

define('glColor3uiv', dlsym(libGL, 'glColor3uiv'), null, 'void', 'void *');
export function glColor3uiv(v) {
  return call('glColor3uiv', v);
}

define('glColor3us', dlsym(libGL, 'glColor3us'), null, 'void', 'uint16', 'uint16', 'uint16');
export function glColor3us(red, green, blue) {
  return call('glColor3us', red, green, blue);
}

define('glColor3usv', dlsym(libGL, 'glColor3usv'), null, 'void', 'void *');
export function glColor3usv(v) {
  return call('glColor3usv', v);
}

define('glColor4b', dlsym(libGL, 'glColor4b'), null, 'void', 'sint8', 'sint8', 'sint8', 'sint8');
export function glColor4b(red, green, blue, alpha) {
  return call('glColor4b', red, green, blue, alpha);
}

define('glColor4bv', dlsym(libGL, 'glColor4bv'), null, 'void', 'void *');
export function glColor4bv(v) {
  return call('glColor4bv', v);
}

define('glColor4d', dlsym(libGL, 'glColor4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glColor4d(red, green, blue, alpha) {
  return call('glColor4d', red, green, blue, alpha);
}

define('glColor4dv', dlsym(libGL, 'glColor4dv'), null, 'void', 'void *');
export function glColor4dv(v) {
  return call('glColor4dv', v);
}

define('glColor4f', dlsym(libGL, 'glColor4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glColor4f(red, green, blue, alpha) {
  return call('glColor4f', red, green, blue, alpha);
}

define('glColor4fv', dlsym(libGL, 'glColor4fv'), null, 'void', 'void *');
export function glColor4fv(v) {
  return call('glColor4fv', v);
}

define('glColor4i', dlsym(libGL, 'glColor4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glColor4i(red, green, blue, alpha) {
  return call('glColor4i', red, green, blue, alpha);
}

define('glColor4iv', dlsym(libGL, 'glColor4iv'), null, 'void', 'void *');
export function glColor4iv(v) {
  return call('glColor4iv', v);
}

define('glColor4s', dlsym(libGL, 'glColor4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glColor4s(red, green, blue, alpha) {
  return call('glColor4s', red, green, blue, alpha);
}

define('glColor4sv', dlsym(libGL, 'glColor4sv'), null, 'void', 'void *');
export function glColor4sv(v) {
  return call('glColor4sv', v);
}

define(
  'glColor4ub',
  dlsym(libGL, 'glColor4ub'),
  null,
  'void',
  'unsigned char',
  'unsigned char',
  'unsigned char',
  'unsigned char'
);
export function glColor4ub(red, green, blue, alpha) {
  return call('glColor4ub', red, green, blue, alpha);
}

define('glColor4ubv', dlsym(libGL, 'glColor4ubv'), null, 'void', 'void *');
export function glColor4ubv(v) {
  return call('glColor4ubv', v);
}

define(
  'glColor4ui',
  dlsym(libGL, 'glColor4ui'),
  null,
  'void',
  'unsigned int',
  'unsigned int',
  'unsigned int',
  'unsigned int'
);
export function glColor4ui(red, green, blue, alpha) {
  return call('glColor4ui', red, green, blue, alpha);
}

define('glColor4uiv', dlsym(libGL, 'glColor4uiv'), null, 'void', 'void *');
export function glColor4uiv(v) {
  return call('glColor4uiv', v);
}

define('glColor4us', dlsym(libGL, 'glColor4us'), null, 'void', 'uint16', 'uint16', 'uint16', 'uint16');
export function glColor4us(red, green, blue, alpha) {
  return call('glColor4us', red, green, blue, alpha);
}

define('glColor4usv', dlsym(libGL, 'glColor4usv'), null, 'void', 'void *');
export function glColor4usv(v) {
  return call('glColor4usv', v);
}

define(
  'glColorMask',
  dlsym(libGL, 'glColorMask'),
  null,
  'void',
  'unsigned char',
  'unsigned char',
  'unsigned char',
  'unsigned char'
);
export function glColorMask(red, green, blue, alpha) {
  return call('glColorMask', red, green, blue, alpha);
}

define('glColorMaterial', dlsym(libGL, 'glColorMaterial'), null, 'void', 'unsigned int', 'unsigned int');
export function glColorMaterial(face, mode) {
  return call('glColorMaterial', face, mode);
}

define('glColorPointer', dlsym(libGL, 'glColorPointer'), null, 'void', 'int', 'unsigned int', 'int', 'void *');
export function glColorPointer(size, type, stride, pointer) {
  return call('glColorPointer', size, type, stride, pointer);
}

define('glCopyPixels', dlsym(libGL, 'glCopyPixels'), null, 'void', 'int', 'int', 'int', 'int', 'unsigned int');
export function glCopyPixels(x, y, width, height, type) {
  return call('glCopyPixels', x, y, width, height, type);
}

define(
  'glCopyTexImage1D',
  dlsym(libGL, 'glCopyTexImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexImage1D(target, level, internalFormat, x, y, width, border) {
  return call('glCopyTexImage1D', target, level, internalFormat, x, y, width, border);
}

define(
  'glCopyTexImage2D',
  dlsym(libGL, 'glCopyTexImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexImage2D(target, level, internalFormat, x, y, width, height, border) {
  return call('glCopyTexImage2D', target, level, internalFormat, x, y, width, height, border);
}

define(
  'glCopyTexSubImage1D',
  dlsym(libGL, 'glCopyTexSubImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexSubImage1D(target, level, xoffset, x, y, width) {
  return call('glCopyTexSubImage1D', target, level, xoffset, x, y, width);
}

define(
  'glCopyTexSubImage2D',
  dlsym(libGL, 'glCopyTexSubImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'int'
);
export function glCopyTexSubImage2D(target, level, xoffset, yoffset, x, y, width, height) {
  return call('glCopyTexSubImage2D', target, level, xoffset, yoffset, x, y, width, height);
}

define('glCullFace', dlsym(libGL, 'glCullFace'), null, 'void', 'unsigned int');
export function glCullFace(mode) {
  return call('glCullFace', mode);
}

define('glDeleteLists', dlsym(libGL, 'glDeleteLists'), null, 'void', 'unsigned int', 'int');
export function glDeleteLists(list, range) {
  return call('glDeleteLists', list, range);
}

define('glDeleteTextures', dlsym(libGL, 'glDeleteTextures'), null, 'void', 'int', 'void *');
export function glDeleteTextures(n, textures) {
  return call('glDeleteTextures', n, textures);
}

define('glDepthFunc', dlsym(libGL, 'glDepthFunc'), null, 'void', 'unsigned int');
export function glDepthFunc(func) {
  return call('glDepthFunc', func);
}

define('glDepthMask', dlsym(libGL, 'glDepthMask'), null, 'void', 'unsigned char');
export function glDepthMask(flag) {
  return call('glDepthMask', flag);
}

define('glDepthRange', dlsym(libGL, 'glDepthRange'), null, 'void', 'double', 'double');
export function glDepthRange(zNear, zFar) {
  return call('glDepthRange', zNear, zFar);
}

define('glDisable', dlsym(libGL, 'glDisable'), null, 'void', 'unsigned int');
export function glDisable(cap) {
  return call('glDisable', cap);
}

define('glDisableClientState', dlsym(libGL, 'glDisableClientState'), null, 'void', 'unsigned int');
export function glDisableClientState(array) {
  return call('glDisableClientState', array);
}

define('glDrawArrays', dlsym(libGL, 'glDrawArrays'), null, 'void', 'unsigned int', 'int', 'int');
export function glDrawArrays(mode, first, count) {
  return call('glDrawArrays', mode, first, count);
}

define('glDrawBuffer', dlsym(libGL, 'glDrawBuffer'), null, 'void', 'unsigned int');
export function glDrawBuffer(mode) {
  return call('glDrawBuffer', mode);
}

define('glDrawElements', dlsym(libGL, 'glDrawElements'), null, 'void', 'unsigned int', 'int', 'unsigned int', 'void *');
export function glDrawElements(mode, count, type, indices) {
  return call('glDrawElements', mode, count, type, indices);
}

define(
  'glDrawPixels',
  dlsym(libGL, 'glDrawPixels'),
  null,
  'void',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glDrawPixels(width, height, format, type, pixels) {
  return call('glDrawPixels', width, height, format, type, pixels);
}

define('glEdgeFlag', dlsym(libGL, 'glEdgeFlag'), null, 'void', 'unsigned char');
export function glEdgeFlag(flag) {
  return call('glEdgeFlag', flag);
}

define('glEdgeFlagPointer', dlsym(libGL, 'glEdgeFlagPointer'), null, 'void', 'int', 'void *');
export function glEdgeFlagPointer(stride, pointer) {
  return call('glEdgeFlagPointer', stride, pointer);
}

define('glEdgeFlagv', dlsym(libGL, 'glEdgeFlagv'), null, 'void', 'void *');
export function glEdgeFlagv(flag) {
  return call('glEdgeFlagv', flag);
}

define('glEnable', dlsym(libGL, 'glEnable'), null, 'void', 'unsigned int');
export function glEnable(cap) {
  return call('glEnable', cap);
}

define('glEnableClientState', dlsym(libGL, 'glEnableClientState'), null, 'void', 'unsigned int');
export function glEnableClientState(array) {
  return call('glEnableClientState', array);
}

define('glEvalCoord1d', dlsym(libGL, 'glEvalCoord1d'), null, 'void', 'double');
export function glEvalCoord1d(u) {
  return call('glEvalCoord1d', u);
}

define('glEvalCoord1dv', dlsym(libGL, 'glEvalCoord1dv'), null, 'void', 'void *');
export function glEvalCoord1dv(u) {
  return call('glEvalCoord1dv', u);
}

define('glEvalCoord1f', dlsym(libGL, 'glEvalCoord1f'), null, 'void', 'float');
export function glEvalCoord1f(u) {
  return call('glEvalCoord1f', u);
}

define('glEvalCoord1fv', dlsym(libGL, 'glEvalCoord1fv'), null, 'void', 'void *');
export function glEvalCoord1fv(u) {
  return call('glEvalCoord1fv', u);
}

define('glEvalCoord2d', dlsym(libGL, 'glEvalCoord2d'), null, 'void', 'double', 'double');
export function glEvalCoord2d(u, v) {
  return call('glEvalCoord2d', u, v);
}

define('glEvalCoord2dv', dlsym(libGL, 'glEvalCoord2dv'), null, 'void', 'void *');
export function glEvalCoord2dv(u) {
  return call('glEvalCoord2dv', u);
}

define('glEvalCoord2f', dlsym(libGL, 'glEvalCoord2f'), null, 'void', 'float', 'float');
export function glEvalCoord2f(u, v) {
  return call('glEvalCoord2f', u, v);
}

define('glEvalCoord2fv', dlsym(libGL, 'glEvalCoord2fv'), null, 'void', 'void *');
export function glEvalCoord2fv(u) {
  return call('glEvalCoord2fv', u);
}

define('glEvalMesh1', dlsym(libGL, 'glEvalMesh1'), null, 'void', 'unsigned int', 'int', 'int');
export function glEvalMesh1(mode, i1, i2) {
  return call('glEvalMesh1', mode, i1, i2);
}

define('glEvalMesh2', dlsym(libGL, 'glEvalMesh2'), null, 'void', 'unsigned int', 'int', 'int', 'int', 'int');
export function glEvalMesh2(mode, i1, i2, j1, j2) {
  return call('glEvalMesh2', mode, i1, i2, j1, j2);
}

define('glEvalPoint1', dlsym(libGL, 'glEvalPoint1'), null, 'void', 'int');
export function glEvalPoint1(i) {
  return call('glEvalPoint1', i);
}

define('glEvalPoint2', dlsym(libGL, 'glEvalPoint2'), null, 'void', 'int', 'int');
export function glEvalPoint2(i, j) {
  return call('glEvalPoint2', i, j);
}

define('glFeedbackBuffer', dlsym(libGL, 'glFeedbackBuffer'), null, 'void', 'int', 'unsigned int', 'void *');
export function glFeedbackBuffer(size, type, buffer) {
  return call('glFeedbackBuffer', size, type, buffer);
}

define('glFogf', dlsym(libGL, 'glFogf'), null, 'void', 'unsigned int', 'float');
export function glFogf(pname, param) {
  return call('glFogf', pname, param);
}

define('glFogfv', dlsym(libGL, 'glFogfv'), null, 'void', 'unsigned int', 'void *');
export function glFogfv(pname, params) {
  return call('glFogfv', pname, params);
}

define('glFogi', dlsym(libGL, 'glFogi'), null, 'void', 'unsigned int', 'int');
export function glFogi(pname, param) {
  return call('glFogi', pname, param);
}

define('glFogiv', dlsym(libGL, 'glFogiv'), null, 'void', 'unsigned int', 'void *');
export function glFogiv(pname, params) {
  return call('glFogiv', pname, params);
}

define('glFrontFace', dlsym(libGL, 'glFrontFace'), null, 'void', 'unsigned int');
export function glFrontFace(mode) {
  return call('glFrontFace', mode);
}

define(
  'glFrustum',
  dlsym(libGL, 'glFrustum'),
  null,
  'void',
  'double',
  'double',
  'double',
  'double',
  'double',
  'double'
);
export function glFrustum(left, right, bottom, top, zNear, zFar) {
  return call('glFrustum', left, right, bottom, top, zNear, zFar);
}

define('glGenLists', dlsym(libGL, 'glGenLists'), null, 'unsigned int', 'int');
export function glGenLists(range) {
  return call('glGenLists', range);
}

define('glGenTextures', dlsym(libGL, 'glGenTextures'), null, 'void', 'int', 'void *');
export function glGenTextures(n, textures) {
  return call('glGenTextures', n, textures);
}

define('glGetBooleanv', dlsym(libGL, 'glGetBooleanv'), null, 'void', 'unsigned int', 'void *');
export function glGetBooleanv(pname, params) {
  return call('glGetBooleanv', pname, params);
}

define('glGetClipPlane', dlsym(libGL, 'glGetClipPlane'), null, 'void', 'unsigned int', 'void *');
export function glGetClipPlane(plane, equation) {
  return call('glGetClipPlane', plane, equation);
}

define('glGetDoublev', dlsym(libGL, 'glGetDoublev'), null, 'void', 'unsigned int', 'void *');
export function glGetDoublev(pname, params) {
  return call('glGetDoublev', pname, params);
}

define('glGetFloatv', dlsym(libGL, 'glGetFloatv'), null, 'void', 'unsigned int', 'void *');
export function glGetFloatv(pname, params) {
  return call('glGetFloatv', pname, params);
}

define('glGetIntegerv', dlsym(libGL, 'glGetIntegerv'), null, 'void', 'unsigned int', 'void *');
export function glGetIntegerv(pname, params) {
  return call('glGetIntegerv', pname, params);
}

define('glGetLightfv', dlsym(libGL, 'glGetLightfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetLightfv(light, pname, params) {
  return call('glGetLightfv', light, pname, params);
}

define('glGetLightiv', dlsym(libGL, 'glGetLightiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetLightiv(light, pname, params) {
  return call('glGetLightiv', light, pname, params);
}

define('glGetMapdv', dlsym(libGL, 'glGetMapdv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMapdv(target, query, v) {
  return call('glGetMapdv', target, query, v);
}

define('glGetMapfv', dlsym(libGL, 'glGetMapfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMapfv(target, query, v) {
  return call('glGetMapfv', target, query, v);
}

define('glGetMapiv', dlsym(libGL, 'glGetMapiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMapiv(target, query, v) {
  return call('glGetMapiv', target, query, v);
}

define('glGetMaterialfv', dlsym(libGL, 'glGetMaterialfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMaterialfv(face, pname, params) {
  return call('glGetMaterialfv', face, pname, params);
}

define('glGetMaterialiv', dlsym(libGL, 'glGetMaterialiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetMaterialiv(face, pname, params) {
  return call('glGetMaterialiv', face, pname, params);
}

define('glGetPixelMapfv', dlsym(libGL, 'glGetPixelMapfv'), null, 'void', 'unsigned int', 'void *');
export function glGetPixelMapfv(map, values) {
  return call('glGetPixelMapfv', map, values);
}

define('glGetPixelMapuiv', dlsym(libGL, 'glGetPixelMapuiv'), null, 'void', 'unsigned int', 'void *');
export function glGetPixelMapuiv(map, values) {
  return call('glGetPixelMapuiv', map, values);
}

define('glGetPixelMapusv', dlsym(libGL, 'glGetPixelMapusv'), null, 'void', 'unsigned int', 'void *');
export function glGetPixelMapusv(map, values) {
  return call('glGetPixelMapusv', map, values);
}

define('glGetPointerv', dlsym(libGL, 'glGetPointerv'), null, 'void', 'unsigned int', 'void *');
export function glGetPointerv(pname, params) {
  return call('glGetPointerv', pname, params);
}

define('glGetPolygonStipple', dlsym(libGL, 'glGetPolygonStipple'), null, 'void', 'void *');
export function glGetPolygonStipple(mask) {
  return call('glGetPolygonStipple', mask);
}

define('glGetString', dlsym(libGL, 'glGetString'), null, 'void *', 'unsigned int');
export function glGetString(name) {
  return call('glGetString', name);
}

define('glGetTexEnvfv', dlsym(libGL, 'glGetTexEnvfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexEnvfv(target, pname, params) {
  return call('glGetTexEnvfv', target, pname, params);
}

define('glGetTexEnviv', dlsym(libGL, 'glGetTexEnviv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexEnviv(target, pname, params) {
  return call('glGetTexEnviv', target, pname, params);
}

define('glGetTexGendv', dlsym(libGL, 'glGetTexGendv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexGendv(coord, pname, params) {
  return call('glGetTexGendv', coord, pname, params);
}

define('glGetTexGenfv', dlsym(libGL, 'glGetTexGenfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexGenfv(coord, pname, params) {
  return call('glGetTexGenfv', coord, pname, params);
}

define('glGetTexGeniv', dlsym(libGL, 'glGetTexGeniv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glGetTexGeniv(coord, pname, params) {
  return call('glGetTexGeniv', coord, pname, params);
}

define(
  'glGetTexImage',
  dlsym(libGL, 'glGetTexImage'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glGetTexImage(target, level, format, type, pixels) {
  return call('glGetTexImage', target, level, format, type, pixels);
}

define(
  'glGetTexLevelParameterfv',
  dlsym(libGL, 'glGetTexLevelParameterfv'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'void *'
);
export function glGetTexLevelParameterfv(target, level, pname, params) {
  return call('glGetTexLevelParameterfv', target, level, pname, params);
}

define(
  'glGetTexLevelParameteriv',
  dlsym(libGL, 'glGetTexLevelParameteriv'),
  null,
  'void',
  'unsigned int',
  'int',
  'unsigned int',
  'void *'
);
export function glGetTexLevelParameteriv(target, level, pname, params) {
  return call('glGetTexLevelParameteriv', target, level, pname, params);
}

define(
  'glGetTexParameterfv',
  dlsym(libGL, 'glGetTexParameterfv'),
  null,
  'void',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glGetTexParameterfv(target, pname, params) {
  return call('glGetTexParameterfv', target, pname, params);
}

define(
  'glGetTexParameteriv',
  dlsym(libGL, 'glGetTexParameteriv'),
  null,
  'void',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glGetTexParameteriv(target, pname, params) {
  return call('glGetTexParameteriv', target, pname, params);
}

define('glHint', dlsym(libGL, 'glHint'), null, 'void', 'unsigned int', 'unsigned int');
export function glHint(target, mode) {
  return call('glHint', target, mode);
}

define('glIndexd', dlsym(libGL, 'glIndexd'), null, 'void', 'double');
export function glIndexd(c) {
  return call('glIndexd', c);
}

define('glIndexdv', dlsym(libGL, 'glIndexdv'), null, 'void', 'void *');
export function glIndexdv(c) {
  return call('glIndexdv', c);
}

define('glIndexf', dlsym(libGL, 'glIndexf'), null, 'void', 'float');
export function glIndexf(c) {
  return call('glIndexf', c);
}

define('glIndexfv', dlsym(libGL, 'glIndexfv'), null, 'void', 'void *');
export function glIndexfv(c) {
  return call('glIndexfv', c);
}

define('glIndexi', dlsym(libGL, 'glIndexi'), null, 'void', 'int');
export function glIndexi(c) {
  return call('glIndexi', c);
}

define('glIndexiv', dlsym(libGL, 'glIndexiv'), null, 'void', 'void *');
export function glIndexiv(c) {
  return call('glIndexiv', c);
}

define('glIndexMask', dlsym(libGL, 'glIndexMask'), null, 'void', 'unsigned int');
export function glIndexMask(mask) {
  return call('glIndexMask', mask);
}

define('glIndexPointer', dlsym(libGL, 'glIndexPointer'), null, 'void', 'unsigned int', 'int', 'void *');
export function glIndexPointer(type, stride, pointer) {
  return call('glIndexPointer', type, stride, pointer);
}

define('glIndexs', dlsym(libGL, 'glIndexs'), null, 'void', 'short');
export function glIndexs(c) {
  return call('glIndexs', c);
}

define('glIndexsv', dlsym(libGL, 'glIndexsv'), null, 'void', 'void *');
export function glIndexsv(c) {
  return call('glIndexsv', c);
}

define('glIndexub', dlsym(libGL, 'glIndexub'), null, 'void', 'unsigned char');
export function glIndexub(c) {
  return call('glIndexub', c);
}

define('glIndexubv', dlsym(libGL, 'glIndexubv'), null, 'void', 'void *');
export function glIndexubv(c) {
  return call('glIndexubv', c);
}

define('glInterleavedArrays', dlsym(libGL, 'glInterleavedArrays'), null, 'void', 'unsigned int', 'int', 'void *');
export function glInterleavedArrays(format, stride, pointer) {
  return call('glInterleavedArrays', format, stride, pointer);
}

define('glIsEnabled', dlsym(libGL, 'glIsEnabled'), null, 'unsigned char', 'unsigned int');
export function glIsEnabled(cap) {
  return call('glIsEnabled', cap);
}

define('glIsList', dlsym(libGL, 'glIsList'), null, 'unsigned char', 'unsigned int');
export function glIsList(list) {
  return call('glIsList', list);
}

define('glIsTexture', dlsym(libGL, 'glIsTexture'), null, 'unsigned char', 'unsigned int');
export function glIsTexture(texture) {
  return call('glIsTexture', texture);
}

define('glLightf', dlsym(libGL, 'glLightf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glLightf(light, pname, param) {
  return call('glLightf', light, pname, param);
}

define('glLightfv', dlsym(libGL, 'glLightfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glLightfv(light, pname, params) {
  return call('glLightfv', light, pname, params);
}

define('glLighti', dlsym(libGL, 'glLighti'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glLighti(light, pname, param) {
  return call('glLighti', light, pname, param);
}

define('glLightiv', dlsym(libGL, 'glLightiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glLightiv(light, pname, params) {
  return call('glLightiv', light, pname, params);
}

define('glLightModelf', dlsym(libGL, 'glLightModelf'), null, 'void', 'unsigned int', 'float');
export function glLightModelf(pname, param) {
  return call('glLightModelf', pname, param);
}

define('glLightModelfv', dlsym(libGL, 'glLightModelfv'), null, 'void', 'unsigned int', 'void *');
export function glLightModelfv(pname, params) {
  return call('glLightModelfv', pname, params);
}

define('glLightModeli', dlsym(libGL, 'glLightModeli'), null, 'void', 'unsigned int', 'int');
export function glLightModeli(pname, param) {
  return call('glLightModeli', pname, param);
}

define('glLightModeliv', dlsym(libGL, 'glLightModeliv'), null, 'void', 'unsigned int', 'void *');
export function glLightModeliv(pname, params) {
  return call('glLightModeliv', pname, params);
}

define('glLineStipple', dlsym(libGL, 'glLineStipple'), null, 'void', 'int', 'uint16');
export function glLineStipple(factor, pattern) {
  return call('glLineStipple', factor, pattern);
}

define('glLineWidth', dlsym(libGL, 'glLineWidth'), null, 'void', 'float');
export function glLineWidth(width) {
  return call('glLineWidth', width);
}

define('glListBase', dlsym(libGL, 'glListBase'), null, 'void', 'unsigned int');
export function glListBase(base) {
  return call('glListBase', base);
}

define('glLoadMatrixd', dlsym(libGL, 'glLoadMatrixd'), null, 'void', 'void *');
export function glLoadMatrixd(m) {
  return call('glLoadMatrixd', m);
}

define('glLoadMatrixf', dlsym(libGL, 'glLoadMatrixf'), null, 'void', 'void *');
export function glLoadMatrixf(m) {
  return call('glLoadMatrixf', m);
}

define('glLoadName', dlsym(libGL, 'glLoadName'), null, 'void', 'unsigned int');
export function glLoadName(name) {
  return call('glLoadName', name);
}

define('glLogicOp', dlsym(libGL, 'glLogicOp'), null, 'void', 'unsigned int');
export function glLogicOp(opcode) {
  return call('glLogicOp', opcode);
}

define('glMap1d', dlsym(libGL, 'glMap1d'), null, 'void', 'unsigned int', 'double', 'double', 'int', 'int', 'void *');
export function glMap1d(target, u1, u2, stride, order, points) {
  return call('glMap1d', target, u1, u2, stride, order, points);
}

define('glMap1f', dlsym(libGL, 'glMap1f'), null, 'void', 'unsigned int', 'float', 'float', 'int', 'int', 'void *');
export function glMap1f(target, u1, u2, stride, order, points) {
  return call('glMap1f', target, u1, u2, stride, order, points);
}

define(
  'glMap2d',
  dlsym(libGL, 'glMap2d'),
  null,
  'void',
  'unsigned int',
  'double',
  'double',
  'int',
  'int',
  'double',
  'double',
  'int',
  'int',
  'void *'
);
export function glMap2d(target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points) {
  return call('glMap2d', target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points);
}

define(
  'glMap2f',
  dlsym(libGL, 'glMap2f'),
  null,
  'void',
  'unsigned int',
  'float',
  'float',
  'int',
  'int',
  'float',
  'float',
  'int',
  'int',
  'void *'
);
export function glMap2f(target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points) {
  return call('glMap2f', target, u1, u2, ustride, uorder, v1, v2, vstride, vorder, points);
}

define('glMapGrid1d', dlsym(libGL, 'glMapGrid1d'), null, 'void', 'int', 'double', 'double');
export function glMapGrid1d(un, u1, u2) {
  return call('glMapGrid1d', un, u1, u2);
}

define('glMapGrid1f', dlsym(libGL, 'glMapGrid1f'), null, 'void', 'int', 'float', 'float');
export function glMapGrid1f(un, u1, u2) {
  return call('glMapGrid1f', un, u1, u2);
}

define('glMapGrid2d', dlsym(libGL, 'glMapGrid2d'), null, 'void', 'int', 'double', 'double', 'int', 'double', 'double');
export function glMapGrid2d(un, u1, u2, vn, v1, v2) {
  return call('glMapGrid2d', un, u1, u2, vn, v1, v2);
}

define('glMapGrid2f', dlsym(libGL, 'glMapGrid2f'), null, 'void', 'int', 'float', 'float', 'int', 'float', 'float');
export function glMapGrid2f(un, u1, u2, vn, v1, v2) {
  return call('glMapGrid2f', un, u1, u2, vn, v1, v2);
}

define('glMaterialf', dlsym(libGL, 'glMaterialf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glMaterialf(face, pname, param) {
  return call('glMaterialf', face, pname, param);
}

define('glMaterialfv', dlsym(libGL, 'glMaterialfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glMaterialfv(face, pname, params) {
  return call('glMaterialfv', face, pname, params);
}

define('glMateriali', dlsym(libGL, 'glMateriali'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glMateriali(face, pname, param) {
  return call('glMateriali', face, pname, param);
}

define('glMaterialiv', dlsym(libGL, 'glMaterialiv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glMaterialiv(face, pname, params) {
  return call('glMaterialiv', face, pname, params);
}

define('glMatrixMode', dlsym(libGL, 'glMatrixMode'), null, 'void', 'unsigned int');
export function glMatrixMode(mode) {
  return call('glMatrixMode', mode);
}

define('glMultMatrixd', dlsym(libGL, 'glMultMatrixd'), null, 'void', 'void *');
export function glMultMatrixd(m) {
  return call('glMultMatrixd', m);
}

define('glMultMatrixf', dlsym(libGL, 'glMultMatrixf'), null, 'void', 'void *');
export function glMultMatrixf(m) {
  return call('glMultMatrixf', m);
}

define('glNewList', dlsym(libGL, 'glNewList'), null, 'void', 'unsigned int', 'unsigned int');
export function glNewList(list, mode) {
  return call('glNewList', list, mode);
}

define('glNormal3b', dlsym(libGL, 'glNormal3b'), null, 'void', 'sint8', 'sint8', 'sint8');
export function glNormal3b(nx, ny, nz) {
  return call('glNormal3b', nx, ny, nz);
}

define('glNormal3bv', dlsym(libGL, 'glNormal3bv'), null, 'void', 'void *');
export function glNormal3bv(v) {
  return call('glNormal3bv', v);
}

define('glNormal3d', dlsym(libGL, 'glNormal3d'), null, 'void', 'double', 'double', 'double');
export function glNormal3d(nx, ny, nz) {
  return call('glNormal3d', nx, ny, nz);
}

define('glNormal3dv', dlsym(libGL, 'glNormal3dv'), null, 'void', 'void *');
export function glNormal3dv(v) {
  return call('glNormal3dv', v);
}

define('glNormal3f', dlsym(libGL, 'glNormal3f'), null, 'void', 'float', 'float', 'float');
export function glNormal3f(nx, ny, nz) {
  return call('glNormal3f', nx, ny, nz);
}

define('glNormal3fv', dlsym(libGL, 'glNormal3fv'), null, 'void', 'void *');
export function glNormal3fv(v) {
  return call('glNormal3fv', v);
}

define('glNormal3i', dlsym(libGL, 'glNormal3i'), null, 'void', 'int', 'int', 'int');
export function glNormal3i(nx, ny, nz) {
  return call('glNormal3i', nx, ny, nz);
}

define('glNormal3iv', dlsym(libGL, 'glNormal3iv'), null, 'void', 'void *');
export function glNormal3iv(v) {
  return call('glNormal3iv', v);
}

define('glNormal3s', dlsym(libGL, 'glNormal3s'), null, 'void', 'short', 'short', 'short');
export function glNormal3s(nx, ny, nz) {
  return call('glNormal3s', nx, ny, nz);
}

define('glNormal3sv', dlsym(libGL, 'glNormal3sv'), null, 'void', 'void *');
export function glNormal3sv(v) {
  return call('glNormal3sv', v);
}

define('glNormalPointer', dlsym(libGL, 'glNormalPointer'), null, 'void', 'unsigned int', 'int', 'void *');
export function glNormalPointer(type, stride, pointer) {
  return call('glNormalPointer', type, stride, pointer);
}

define('glOrtho', dlsym(libGL, 'glOrtho'), null, 'void', 'double', 'double', 'double', 'double', 'double', 'double');
export function glOrtho(left, right, bottom, top, zNear, zFar) {
  return call('glOrtho', left, right, bottom, top, zNear, zFar);
}

define('glPassThrough', dlsym(libGL, 'glPassThrough'), null, 'void', 'float');
export function glPassThrough(token) {
  return call('glPassThrough', token);
}

define('glPixelMapfv', dlsym(libGL, 'glPixelMapfv'), null, 'void', 'unsigned int', 'int', 'void *');
export function glPixelMapfv(map, mapsize, values) {
  return call('glPixelMapfv', map, mapsize, values);
}

define('glPixelMapuiv', dlsym(libGL, 'glPixelMapuiv'), null, 'void', 'unsigned int', 'int', 'void *');
export function glPixelMapuiv(map, mapsize, values) {
  return call('glPixelMapuiv', map, mapsize, values);
}

define('glPixelMapusv', dlsym(libGL, 'glPixelMapusv'), null, 'void', 'unsigned int', 'int', 'void *');
export function glPixelMapusv(map, mapsize, values) {
  return call('glPixelMapusv', map, mapsize, values);
}

define('glPixelStoref', dlsym(libGL, 'glPixelStoref'), null, 'void', 'unsigned int', 'float');
export function glPixelStoref(pname, param) {
  return call('glPixelStoref', pname, param);
}

define('glPixelStorei', dlsym(libGL, 'glPixelStorei'), null, 'void', 'unsigned int', 'int');
export function glPixelStorei(pname, param) {
  return call('glPixelStorei', pname, param);
}

define('glPixelTransferf', dlsym(libGL, 'glPixelTransferf'), null, 'void', 'unsigned int', 'float');
export function glPixelTransferf(pname, param) {
  return call('glPixelTransferf', pname, param);
}

define('glPixelTransferi', dlsym(libGL, 'glPixelTransferi'), null, 'void', 'unsigned int', 'int');
export function glPixelTransferi(pname, param) {
  return call('glPixelTransferi', pname, param);
}

define('glPixelZoom', dlsym(libGL, 'glPixelZoom'), null, 'void', 'float', 'float');
export function glPixelZoom(xfactor, yfactor) {
  return call('glPixelZoom', xfactor, yfactor);
}

define('glPointSize', dlsym(libGL, 'glPointSize'), null, 'void', 'float');
export function glPointSize(size) {
  return call('glPointSize', size);
}

define('glPolygonMode', dlsym(libGL, 'glPolygonMode'), null, 'void', 'unsigned int', 'unsigned int');
export function glPolygonMode(face, mode) {
  return call('glPolygonMode', face, mode);
}

define('glPolygonOffset', dlsym(libGL, 'glPolygonOffset'), null, 'void', 'float', 'float');
export function glPolygonOffset(factor, units) {
  return call('glPolygonOffset', factor, units);
}

define('glPolygonStipple', dlsym(libGL, 'glPolygonStipple'), null, 'void', 'void *');
export function glPolygonStipple(mask) {
  return call('glPolygonStipple', mask);
}

define('glPrioritizeTextures', dlsym(libGL, 'glPrioritizeTextures'), null, 'void', 'int', 'void *', 'void *');
export function glPrioritizeTextures(n, textures, priorities) {
  return call('glPrioritizeTextures', n, textures, priorities);
}

define('glPushAttrib', dlsym(libGL, 'glPushAttrib'), null, 'void', 'unsigned int');
export function glPushAttrib(mask) {
  return call('glPushAttrib', mask);
}

define('glPushClientAttrib', dlsym(libGL, 'glPushClientAttrib'), null, 'void', 'unsigned int');
export function glPushClientAttrib(mask) {
  return call('glPushClientAttrib', mask);
}

define('glPushName', dlsym(libGL, 'glPushName'), null, 'void', 'unsigned int');
export function glPushName(name) {
  return call('glPushName', name);
}

define('glRasterPos2d', dlsym(libGL, 'glRasterPos2d'), null, 'void', 'double', 'double');
export function glRasterPos2d(x, y) {
  return call('glRasterPos2d', x, y);
}

define('glRasterPos2dv', dlsym(libGL, 'glRasterPos2dv'), null, 'void', 'void *');
export function glRasterPos2dv(v) {
  return call('glRasterPos2dv', v);
}

define('glRasterPos2f', dlsym(libGL, 'glRasterPos2f'), null, 'void', 'float', 'float');
export function glRasterPos2f(x, y) {
  return call('glRasterPos2f', x, y);
}

define('glRasterPos2fv', dlsym(libGL, 'glRasterPos2fv'), null, 'void', 'void *');
export function glRasterPos2fv(v) {
  return call('glRasterPos2fv', v);
}

define('glRasterPos2i', dlsym(libGL, 'glRasterPos2i'), null, 'void', 'int', 'int');
export function glRasterPos2i(x, y) {
  return call('glRasterPos2i', x, y);
}

define('glRasterPos2iv', dlsym(libGL, 'glRasterPos2iv'), null, 'void', 'void *');
export function glRasterPos2iv(v) {
  return call('glRasterPos2iv', v);
}

define('glRasterPos2s', dlsym(libGL, 'glRasterPos2s'), null, 'void', 'short', 'short');
export function glRasterPos2s(x, y) {
  return call('glRasterPos2s', x, y);
}

define('glRasterPos2sv', dlsym(libGL, 'glRasterPos2sv'), null, 'void', 'void *');
export function glRasterPos2sv(v) {
  return call('glRasterPos2sv', v);
}

define('glRasterPos3d', dlsym(libGL, 'glRasterPos3d'), null, 'void', 'double', 'double', 'double');
export function glRasterPos3d(x, y, z) {
  return call('glRasterPos3d', x, y, z);
}

define('glRasterPos3dv', dlsym(libGL, 'glRasterPos3dv'), null, 'void', 'void *');
export function glRasterPos3dv(v) {
  return call('glRasterPos3dv', v);
}

define('glRasterPos3f', dlsym(libGL, 'glRasterPos3f'), null, 'void', 'float', 'float', 'float');
export function glRasterPos3f(x, y, z) {
  return call('glRasterPos3f', x, y, z);
}

define('glRasterPos3fv', dlsym(libGL, 'glRasterPos3fv'), null, 'void', 'void *');
export function glRasterPos3fv(v) {
  return call('glRasterPos3fv', v);
}

define('glRasterPos3i', dlsym(libGL, 'glRasterPos3i'), null, 'void', 'int', 'int', 'int');
export function glRasterPos3i(x, y, z) {
  return call('glRasterPos3i', x, y, z);
}

define('glRasterPos3iv', dlsym(libGL, 'glRasterPos3iv'), null, 'void', 'void *');
export function glRasterPos3iv(v) {
  return call('glRasterPos3iv', v);
}

define('glRasterPos3s', dlsym(libGL, 'glRasterPos3s'), null, 'void', 'short', 'short', 'short');
export function glRasterPos3s(x, y, z) {
  return call('glRasterPos3s', x, y, z);
}

define('glRasterPos3sv', dlsym(libGL, 'glRasterPos3sv'), null, 'void', 'void *');
export function glRasterPos3sv(v) {
  return call('glRasterPos3sv', v);
}

define('glRasterPos4d', dlsym(libGL, 'glRasterPos4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glRasterPos4d(x, y, z, w) {
  return call('glRasterPos4d', x, y, z, w);
}

define('glRasterPos4dv', dlsym(libGL, 'glRasterPos4dv'), null, 'void', 'void *');
export function glRasterPos4dv(v) {
  return call('glRasterPos4dv', v);
}

define('glRasterPos4f', dlsym(libGL, 'glRasterPos4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glRasterPos4f(x, y, z, w) {
  return call('glRasterPos4f', x, y, z, w);
}

define('glRasterPos4fv', dlsym(libGL, 'glRasterPos4fv'), null, 'void', 'void *');
export function glRasterPos4fv(v) {
  return call('glRasterPos4fv', v);
}

define('glRasterPos4i', dlsym(libGL, 'glRasterPos4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glRasterPos4i(x, y, z, w) {
  return call('glRasterPos4i', x, y, z, w);
}

define('glRasterPos4iv', dlsym(libGL, 'glRasterPos4iv'), null, 'void', 'void *');
export function glRasterPos4iv(v) {
  return call('glRasterPos4iv', v);
}

define('glRasterPos4s', dlsym(libGL, 'glRasterPos4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glRasterPos4s(x, y, z, w) {
  return call('glRasterPos4s', x, y, z, w);
}

define('glRasterPos4sv', dlsym(libGL, 'glRasterPos4sv'), null, 'void', 'void *');
export function glRasterPos4sv(v) {
  return call('glRasterPos4sv', v);
}

define('glReadBuffer', dlsym(libGL, 'glReadBuffer'), null, 'void', 'unsigned int');
export function glReadBuffer(mode) {
  return call('glReadBuffer', mode);
}

define(
  'glReadPixels',
  dlsym(libGL, 'glReadPixels'),
  null,
  'void',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glReadPixels(x, y, width, height, format, type, pixels) {
  return call('glReadPixels', x, y, width, height, format, type, pixels);
}

define('glRectd', dlsym(libGL, 'glRectd'), null, 'void', 'double', 'double', 'double', 'double');
export function glRectd(x1, y1, x2, y2) {
  return call('glRectd', x1, y1, x2, y2);
}

define('glRectdv', dlsym(libGL, 'glRectdv'), null, 'void', 'void *', 'void *');
export function glRectdv(v1, v2) {
  return call('glRectdv', v1, v2);
}

define('glRectf', dlsym(libGL, 'glRectf'), null, 'void', 'float', 'float', 'float', 'float');
export function glRectf(x1, y1, x2, y2) {
  return call('glRectf', x1, y1, x2, y2);
}

define('glRectfv', dlsym(libGL, 'glRectfv'), null, 'void', 'void *', 'void *');
export function glRectfv(v1, v2) {
  return call('glRectfv', v1, v2);
}

define('glRecti', dlsym(libGL, 'glRecti'), null, 'void', 'int', 'int', 'int', 'int');
export function glRecti(x1, y1, x2, y2) {
  return call('glRecti', x1, y1, x2, y2);
}

define('glRectiv', dlsym(libGL, 'glRectiv'), null, 'void', 'void *', 'void *');
export function glRectiv(v1, v2) {
  return call('glRectiv', v1, v2);
}

define('glRects', dlsym(libGL, 'glRects'), null, 'void', 'short', 'short', 'short', 'short');
export function glRects(x1, y1, x2, y2) {
  return call('glRects', x1, y1, x2, y2);
}

define('glRectsv', dlsym(libGL, 'glRectsv'), null, 'void', 'void *', 'void *');
export function glRectsv(v1, v2) {
  return call('glRectsv', v1, v2);
}

define('glRenderMode', dlsym(libGL, 'glRenderMode'), null, 'int', 'unsigned int');
export function glRenderMode(mode) {
  return call('glRenderMode', mode);
}

define('glRotated', dlsym(libGL, 'glRotated'), null, 'void', 'double', 'double', 'double', 'double');
export function glRotated(angle, x, y, z) {
  return call('glRotated', angle, x, y, z);
}

define('glRotatef', dlsym(libGL, 'glRotatef'), null, 'void', 'float', 'float', 'float', 'float');
export function glRotatef(angle, x, y, z) {
  return call('glRotatef', angle, x, y, z);
}

define('glScaled', dlsym(libGL, 'glScaled'), null, 'void', 'double', 'double', 'double');
export function glScaled(x, y, z) {
  return call('glScaled', x, y, z);
}

define('glScalef', dlsym(libGL, 'glScalef'), null, 'void', 'float', 'float', 'float');
export function glScalef(x, y, z) {
  return call('glScalef', x, y, z);
}

define('glScissor', dlsym(libGL, 'glScissor'), null, 'void', 'int', 'int', 'int', 'int');
export function glScissor(x, y, width, height) {
  return call('glScissor', x, y, width, height);
}

define('glSelectBuffer', dlsym(libGL, 'glSelectBuffer'), null, 'void', 'int', 'void *');
export function glSelectBuffer(size, buffer) {
  return call('glSelectBuffer', size, buffer);
}

define('glShadeModel', dlsym(libGL, 'glShadeModel'), null, 'void', 'unsigned int');
export function glShadeModel(mode) {
  return call('glShadeModel', mode);
}

define('glStencilFunc', dlsym(libGL, 'glStencilFunc'), null, 'void', 'unsigned int', 'int', 'unsigned int');
export function glStencilFunc(func, ref, mask) {
  return call('glStencilFunc', func, ref, mask);
}

define('glStencilMask', dlsym(libGL, 'glStencilMask'), null, 'void', 'unsigned int');
export function glStencilMask(mask) {
  return call('glStencilMask', mask);
}

define('glStencilOp', dlsym(libGL, 'glStencilOp'), null, 'void', 'unsigned int', 'unsigned int', 'unsigned int');
export function glStencilOp(fail, zfail, zpass) {
  return call('glStencilOp', fail, zfail, zpass);
}

define('glTexCoord1d', dlsym(libGL, 'glTexCoord1d'), null, 'void', 'double');
export function glTexCoord1d(s) {
  return call('glTexCoord1d', s);
}

define('glTexCoord1dv', dlsym(libGL, 'glTexCoord1dv'), null, 'void', 'void *');
export function glTexCoord1dv(v) {
  return call('glTexCoord1dv', v);
}

define('glTexCoord1f', dlsym(libGL, 'glTexCoord1f'), null, 'void', 'float');
export function glTexCoord1f(s) {
  return call('glTexCoord1f', s);
}

define('glTexCoord1fv', dlsym(libGL, 'glTexCoord1fv'), null, 'void', 'void *');
export function glTexCoord1fv(v) {
  return call('glTexCoord1fv', v);
}

define('glTexCoord1i', dlsym(libGL, 'glTexCoord1i'), null, 'void', 'int');
export function glTexCoord1i(s) {
  return call('glTexCoord1i', s);
}

define('glTexCoord1iv', dlsym(libGL, 'glTexCoord1iv'), null, 'void', 'void *');
export function glTexCoord1iv(v) {
  return call('glTexCoord1iv', v);
}

define('glTexCoord1s', dlsym(libGL, 'glTexCoord1s'), null, 'void', 'short');
export function glTexCoord1s(s) {
  return call('glTexCoord1s', s);
}

define('glTexCoord1sv', dlsym(libGL, 'glTexCoord1sv'), null, 'void', 'void *');
export function glTexCoord1sv(v) {
  return call('glTexCoord1sv', v);
}

define('glTexCoord2d', dlsym(libGL, 'glTexCoord2d'), null, 'void', 'double', 'double');
export function glTexCoord2d(s, t) {
  return call('glTexCoord2d', s, t);
}

define('glTexCoord2dv', dlsym(libGL, 'glTexCoord2dv'), null, 'void', 'void *');
export function glTexCoord2dv(v) {
  return call('glTexCoord2dv', v);
}

define('glTexCoord2f', dlsym(libGL, 'glTexCoord2f'), null, 'void', 'float', 'float');
export function glTexCoord2f(s, t) {
  return call('glTexCoord2f', s, t);
}

define('glTexCoord2fv', dlsym(libGL, 'glTexCoord2fv'), null, 'void', 'void *');
export function glTexCoord2fv(v) {
  return call('glTexCoord2fv', v);
}

define('glTexCoord2i', dlsym(libGL, 'glTexCoord2i'), null, 'void', 'int', 'int');
export function glTexCoord2i(s, t) {
  return call('glTexCoord2i', s, t);
}

define('glTexCoord2iv', dlsym(libGL, 'glTexCoord2iv'), null, 'void', 'void *');
export function glTexCoord2iv(v) {
  return call('glTexCoord2iv', v);
}

define('glTexCoord2s', dlsym(libGL, 'glTexCoord2s'), null, 'void', 'short', 'short');
export function glTexCoord2s(s, t) {
  return call('glTexCoord2s', s, t);
}

define('glTexCoord2sv', dlsym(libGL, 'glTexCoord2sv'), null, 'void', 'void *');
export function glTexCoord2sv(v) {
  return call('glTexCoord2sv', v);
}

define('glTexCoord3d', dlsym(libGL, 'glTexCoord3d'), null, 'void', 'double', 'double', 'double');
export function glTexCoord3d(s, t, r) {
  return call('glTexCoord3d', s, t, r);
}

define('glTexCoord3dv', dlsym(libGL, 'glTexCoord3dv'), null, 'void', 'void *');
export function glTexCoord3dv(v) {
  return call('glTexCoord3dv', v);
}

define('glTexCoord3f', dlsym(libGL, 'glTexCoord3f'), null, 'void', 'float', 'float', 'float');
export function glTexCoord3f(s, t, r) {
  return call('glTexCoord3f', s, t, r);
}

define('glTexCoord3fv', dlsym(libGL, 'glTexCoord3fv'), null, 'void', 'void *');
export function glTexCoord3fv(v) {
  return call('glTexCoord3fv', v);
}

define('glTexCoord3i', dlsym(libGL, 'glTexCoord3i'), null, 'void', 'int', 'int', 'int');
export function glTexCoord3i(s, t, r) {
  return call('glTexCoord3i', s, t, r);
}

define('glTexCoord3iv', dlsym(libGL, 'glTexCoord3iv'), null, 'void', 'void *');
export function glTexCoord3iv(v) {
  return call('glTexCoord3iv', v);
}

define('glTexCoord3s', dlsym(libGL, 'glTexCoord3s'), null, 'void', 'short', 'short', 'short');
export function glTexCoord3s(s, t, r) {
  return call('glTexCoord3s', s, t, r);
}

define('glTexCoord3sv', dlsym(libGL, 'glTexCoord3sv'), null, 'void', 'void *');
export function glTexCoord3sv(v) {
  return call('glTexCoord3sv', v);
}

define('glTexCoord4d', dlsym(libGL, 'glTexCoord4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glTexCoord4d(s, t, r, q) {
  return call('glTexCoord4d', s, t, r, q);
}

define('glTexCoord4dv', dlsym(libGL, 'glTexCoord4dv'), null, 'void', 'void *');
export function glTexCoord4dv(v) {
  return call('glTexCoord4dv', v);
}

define('glTexCoord4f', dlsym(libGL, 'glTexCoord4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glTexCoord4f(s, t, r, q) {
  return call('glTexCoord4f', s, t, r, q);
}

define('glTexCoord4fv', dlsym(libGL, 'glTexCoord4fv'), null, 'void', 'void *');
export function glTexCoord4fv(v) {
  return call('glTexCoord4fv', v);
}

define('glTexCoord4i', dlsym(libGL, 'glTexCoord4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glTexCoord4i(s, t, r, q) {
  return call('glTexCoord4i', s, t, r, q);
}

define('glTexCoord4iv', dlsym(libGL, 'glTexCoord4iv'), null, 'void', 'void *');
export function glTexCoord4iv(v) {
  return call('glTexCoord4iv', v);
}

define('glTexCoord4s', dlsym(libGL, 'glTexCoord4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glTexCoord4s(s, t, r, q) {
  return call('glTexCoord4s', s, t, r, q);
}

define('glTexCoord4sv', dlsym(libGL, 'glTexCoord4sv'), null, 'void', 'void *');
export function glTexCoord4sv(v) {
  return call('glTexCoord4sv', v);
}

define('glTexCoordPointer', dlsym(libGL, 'glTexCoordPointer'), null, 'void', 'int', 'unsigned int', 'int', 'void *');
export function glTexCoordPointer(size, type, stride, pointer) {
  return call('glTexCoordPointer', size, type, stride, pointer);
}

define('glTexEnvf', dlsym(libGL, 'glTexEnvf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glTexEnvf(target, pname, param) {
  return call('glTexEnvf', target, pname, param);
}

define('glTexEnvfv', dlsym(libGL, 'glTexEnvfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexEnvfv(target, pname, params) {
  return call('glTexEnvfv', target, pname, params);
}

define('glTexEnvi', dlsym(libGL, 'glTexEnvi'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glTexEnvi(target, pname, param) {
  return call('glTexEnvi', target, pname, param);
}

define('glTexEnviv', dlsym(libGL, 'glTexEnviv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexEnviv(target, pname, params) {
  return call('glTexEnviv', target, pname, params);
}

define('glTexGend', dlsym(libGL, 'glTexGend'), null, 'void', 'unsigned int', 'unsigned int', 'double');
export function glTexGend(coord, pname, param) {
  return call('glTexGend', coord, pname, param);
}

define('glTexGendv', dlsym(libGL, 'glTexGendv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexGendv(coord, pname, params) {
  return call('glTexGendv', coord, pname, params);
}

define('glTexGenf', dlsym(libGL, 'glTexGenf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glTexGenf(coord, pname, param) {
  return call('glTexGenf', coord, pname, param);
}

define('glTexGenfv', dlsym(libGL, 'glTexGenfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexGenfv(coord, pname, params) {
  return call('glTexGenfv', coord, pname, params);
}

define('glTexGeni', dlsym(libGL, 'glTexGeni'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glTexGeni(coord, pname, param) {
  return call('glTexGeni', coord, pname, param);
}

define('glTexGeniv', dlsym(libGL, 'glTexGeniv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexGeniv(coord, pname, params) {
  return call('glTexGeniv', coord, pname, params);
}

define(
  'glTexImage1D',
  dlsym(libGL, 'glTexImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexImage1D(target, level, internalformat, width, border, format, type, pixels) {
  return call('glTexImage1D', target, level, internalformat, width, border, format, type, pixels);
}

define(
  'glTexImage2D',
  dlsym(libGL, 'glTexImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexImage2D(target, level, internalformat, width, height, border, format, type, pixels) {
  return call('glTexImage2D', target, level, internalformat, width, height, border, format, type, pixels);
}

define('glTexParameterf', dlsym(libGL, 'glTexParameterf'), null, 'void', 'unsigned int', 'unsigned int', 'float');
export function glTexParameterf(target, pname, param) {
  return call('glTexParameterf', target, pname, param);
}

define('glTexParameterfv', dlsym(libGL, 'glTexParameterfv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexParameterfv(target, pname, params) {
  return call('glTexParameterfv', target, pname, params);
}

define('glTexParameteri', dlsym(libGL, 'glTexParameteri'), null, 'void', 'unsigned int', 'unsigned int', 'int');
export function glTexParameteri(target, pname, param) {
  return call('glTexParameteri', target, pname, param);
}

define('glTexParameteriv', dlsym(libGL, 'glTexParameteriv'), null, 'void', 'unsigned int', 'unsigned int', 'void *');
export function glTexParameteriv(target, pname, params) {
  return call('glTexParameteriv', target, pname, params);
}

define(
  'glTexSubImage1D',
  dlsym(libGL, 'glTexSubImage1D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexSubImage1D(target, level, xoffset, width, format, type, pixels) {
  return call('glTexSubImage1D', target, level, xoffset, width, format, type, pixels);
}

define(
  'glTexSubImage2D',
  dlsym(libGL, 'glTexSubImage2D'),
  null,
  'void',
  'unsigned int',
  'int',
  'int',
  'int',
  'int',
  'int',
  'unsigned int',
  'unsigned int',
  'void *'
);
export function glTexSubImage2D(target, level, xoffset, yoffset, width, height, format, type, pixels) {
  return call('glTexSubImage2D', target, level, xoffset, yoffset, width, height, format, type, pixels);
}

define('glTranslated', dlsym(libGL, 'glTranslated'), null, 'void', 'double', 'double', 'double');
export function glTranslated(x, y, z) {
  return call('glTranslated', x, y, z);
}

define('glTranslatef', dlsym(libGL, 'glTranslatef'), null, 'void', 'float', 'float', 'float');
export function glTranslatef(x, y, z) {
  return call('glTranslatef', x, y, z);
}

define('glVertex2d', dlsym(libGL, 'glVertex2d'), null, 'void', 'double', 'double');
export function glVertex2d(x, y) {
  return call('glVertex2d', x, y);
}

define('glVertex2dv', dlsym(libGL, 'glVertex2dv'), null, 'void', 'void *');
export function glVertex2dv(v) {
  return call('glVertex2dv', v);
}

define('glVertex2f', dlsym(libGL, 'glVertex2f'), null, 'void', 'float', 'float');
export function glVertex2f(x, y) {
  return call('glVertex2f', x, y);
}

define('glVertex2fv', dlsym(libGL, 'glVertex2fv'), null, 'void', 'void *');
export function glVertex2fv(v) {
  return call('glVertex2fv', v);
}

define('glVertex2i', dlsym(libGL, 'glVertex2i'), null, 'void', 'int', 'int');
export function glVertex2i(x, y) {
  return call('glVertex2i', x, y);
}

define('glVertex2iv', dlsym(libGL, 'glVertex2iv'), null, 'void', 'void *');
export function glVertex2iv(v) {
  return call('glVertex2iv', v);
}

define('glVertex2s', dlsym(libGL, 'glVertex2s'), null, 'void', 'short', 'short');
export function glVertex2s(x, y) {
  return call('glVertex2s', x, y);
}

define('glVertex2sv', dlsym(libGL, 'glVertex2sv'), null, 'void', 'void *');
export function glVertex2sv(v) {
  return call('glVertex2sv', v);
}

define('glVertex3d', dlsym(libGL, 'glVertex3d'), null, 'void', 'double', 'double', 'double');
export function glVertex3d(x, y, z) {
  return call('glVertex3d', x, y, z);
}

define('glVertex3dv', dlsym(libGL, 'glVertex3dv'), null, 'void', 'void *');
export function glVertex3dv(v) {
  return call('glVertex3dv', v);
}

define('glVertex3f', dlsym(libGL, 'glVertex3f'), null, 'void', 'float', 'float', 'float');
export function glVertex3f(x, y, z) {
  return call('glVertex3f', x, y, z);
}

define('glVertex3fv', dlsym(libGL, 'glVertex3fv'), null, 'void', 'void *');
export function glVertex3fv(v) {
  return call('glVertex3fv', v);
}

define('glVertex3i', dlsym(libGL, 'glVertex3i'), null, 'void', 'int', 'int', 'int');
export function glVertex3i(x, y, z) {
  return call('glVertex3i', x, y, z);
}

define('glVertex3iv', dlsym(libGL, 'glVertex3iv'), null, 'void', 'void *');
export function glVertex3iv(v) {
  return call('glVertex3iv', v);
}

define('glVertex3s', dlsym(libGL, 'glVertex3s'), null, 'void', 'short', 'short', 'short');
export function glVertex3s(x, y, z) {
  return call('glVertex3s', x, y, z);
}

define('glVertex3sv', dlsym(libGL, 'glVertex3sv'), null, 'void', 'void *');
export function glVertex3sv(v) {
  return call('glVertex3sv', v);
}

define('glVertex4d', dlsym(libGL, 'glVertex4d'), null, 'void', 'double', 'double', 'double', 'double');
export function glVertex4d(x, y, z, w) {
  return call('glVertex4d', x, y, z, w);
}

define('glVertex4dv', dlsym(libGL, 'glVertex4dv'), null, 'void', 'void *');
export function glVertex4dv(v) {
  return call('glVertex4dv', v);
}

define('glVertex4f', dlsym(libGL, 'glVertex4f'), null, 'void', 'float', 'float', 'float', 'float');
export function glVertex4f(x, y, z, w) {
  return call('glVertex4f', x, y, z, w);
}

define('glVertex4fv', dlsym(libGL, 'glVertex4fv'), null, 'void', 'void *');
export function glVertex4fv(v) {
  return call('glVertex4fv', v);
}

define('glVertex4i', dlsym(libGL, 'glVertex4i'), null, 'void', 'int', 'int', 'int', 'int');
export function glVertex4i(x, y, z, w) {
  return call('glVertex4i', x, y, z, w);
}

define('glVertex4iv', dlsym(libGL, 'glVertex4iv'), null, 'void', 'void *');
export function glVertex4iv(v) {
  return call('glVertex4iv', v);
}

define('glVertex4s', dlsym(libGL, 'glVertex4s'), null, 'void', 'short', 'short', 'short', 'short');
export function glVertex4s(x, y, z, w) {
  return call('glVertex4s', x, y, z, w);
}

define('glVertex4sv', dlsym(libGL, 'glVertex4sv'), null, 'void', 'void *');
export function glVertex4sv(v) {
  return call('glVertex4sv', v);
}

define('glVertexPointer', dlsym(libGL, 'glVertexPointer'), null, 'void', 'int', 'unsigned int', 'int', 'void *');
export function glVertexPointer(size, type, stride, pointer) {
  return call('glVertexPointer', size, type, stride, pointer);
}

define('glViewport', dlsym(libGL, 'glViewport'), null, 'void', 'int', 'int', 'int', 'int');
export function glViewport(x, y, width, height) {
  return call('glViewport', x, y, width, height);
}