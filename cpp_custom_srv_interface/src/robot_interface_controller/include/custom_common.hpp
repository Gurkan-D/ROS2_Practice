#pragma once

#define USER_SERVICE_NAME "Device_update"
#define SUBSYSTEM_SERVICE_NAME "Subsystem_update"



enum InterfaceTypes  {Motor,Temperature,System};

static constexpr int en_max_pos = 200;
static constexpr int en_min_pos = 0;