// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/FtAdmittanceParams.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/ft_admittance_params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtAdmittanceParams__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0x04, 0x99, 0x7b, 0x31, 0x60, 0xa6, 0x6c,
      0xd8, 0xb5, 0x19, 0x5d, 0xa7, 0xeb, 0x15, 0x7d,
      0xc8, 0x23, 0x49, 0x3c, 0xc9, 0x7c, 0x87, 0xe8,
      0x98, 0x8a, 0x7c, 0x90, 0xc8, 0xeb, 0xe9, 0xc3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtAdmittanceParams_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xba, 0x11, 0xfd, 0x07, 0xc6, 0x53, 0x75, 0xa1,
      0x34, 0xec, 0x5c, 0xb2, 0x6d, 0x2e, 0x60, 0xa7,
      0xf9, 0x23, 0xd4, 0x35, 0xf2, 0xf2, 0x9c, 0x3e,
      0xe0, 0xb9, 0xd5, 0x5f, 0x98, 0xa2, 0x87, 0x04,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtAdmittanceParams_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd9, 0xf5, 0x7f, 0x56, 0xb4, 0x65, 0xde, 0xdb,
      0x18, 0x3b, 0x1d, 0x30, 0x18, 0xfd, 0xb6, 0x8c,
      0xe3, 0x27, 0xad, 0xee, 0x90, 0x57, 0x3f, 0xb8,
      0x15, 0x02, 0x75, 0xc1, 0xae, 0x7b, 0xa7, 0xc8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtAdmittanceParams_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x00, 0x34, 0xa8, 0x0e, 0x07, 0x23, 0x40, 0xae,
      0xbd, 0x54, 0xbf, 0xf5, 0x80, 0xec, 0x58, 0x47,
      0x31, 0xe2, 0xff, 0x48, 0x16, 0x13, 0xeb, 0x72,
      0x95, 0x86, 0x33, 0x77, 0x30, 0xf8, 0xd9, 0xee,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char xarm_msgs__srv__FtAdmittanceParams__TYPE_NAME[] = "xarm_msgs/srv/FtAdmittanceParams";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__FtAdmittanceParams_Event__TYPE_NAME[] = "xarm_msgs/srv/FtAdmittanceParams_Event";
static char xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME[] = "xarm_msgs/srv/FtAdmittanceParams_Request";
static char xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME[] = "xarm_msgs/srv/FtAdmittanceParams_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__FtAdmittanceParams__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__FtAdmittanceParams__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__FtAdmittanceParams__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtAdmittanceParams__FIELDS[] = {
  {
    {xarm_msgs__srv__FtAdmittanceParams__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__FtAdmittanceParams_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__FtAdmittanceParams__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtAdmittanceParams__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtAdmittanceParams__TYPE_NAME, 32, 32},
      {xarm_msgs__srv__FtAdmittanceParams__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__FtAdmittanceParams__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__FtAdmittanceParams_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__FtAdmittanceParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__FtAdmittanceParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__coord[] = "coord";
static char xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__c_axis[] = "c_axis";
static char xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__m[] = "m";
static char xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__k[] = "k";
static char xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__b[] = "b";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtAdmittanceParams_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__coord, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__c_axis, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__m, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__k, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__FIELD_NAME__b, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtAdmittanceParams_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME, 40, 40},
      {xarm_msgs__srv__FtAdmittanceParams_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__FtAdmittanceParams_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__FtAdmittanceParams_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtAdmittanceParams_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__FtAdmittanceParams_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtAdmittanceParams_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME, 41, 41},
      {xarm_msgs__srv__FtAdmittanceParams_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__FtAdmittanceParams_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__FtAdmittanceParams_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__FtAdmittanceParams_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtAdmittanceParams_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__FtAdmittanceParams_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__FtAdmittanceParams_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtAdmittanceParams_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtAdmittanceParams_Event__TYPE_NAME, 38, 38},
      {xarm_msgs__srv__FtAdmittanceParams_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__FtAdmittanceParams_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__FtAdmittanceParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__FtAdmittanceParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - set_ft_sensor_admittance_parameters\n"
  "\n"
  "# task frame (0: base frame. 1: tool frame)\n"
  "int16 coord\n"
  "# a 6d vector of 0s and 1s. 1 means that robot will be admittance in the corresponding axis of the task frame.\n"
  "int16[] c_axis\n"
  "\n"
  "# 6d vector, mass. (kg)\n"
  "float32[] m\n"
  "# 6d vector, stiffness coefficient\n"
  "float32[] k\n"
  "# 6d vector, damping coefficient\n"
  "float32[] b\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtAdmittanceParams__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtAdmittanceParams__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 437, 437},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtAdmittanceParams_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtAdmittanceParams_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtAdmittanceParams_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtAdmittanceParams_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtAdmittanceParams_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtAdmittanceParams_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtAdmittanceParams__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtAdmittanceParams__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__FtAdmittanceParams_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__FtAdmittanceParams_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__FtAdmittanceParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtAdmittanceParams_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtAdmittanceParams_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtAdmittanceParams_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtAdmittanceParams_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtAdmittanceParams_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtAdmittanceParams_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__FtAdmittanceParams_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__FtAdmittanceParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
