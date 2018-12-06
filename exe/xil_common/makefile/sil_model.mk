INCLUDES =  -I${TRICK_HOME}/trick_models \
            -I/usr/local/include/hiredis \
            -I$(TOP_DIR)/models/nxbus/include \
			-I$(TOP_DIR)/models/gnc/include \
			-I$(TOP_DIR)/models/dm/include \
			-I$(TOP_DIR)/models/cad/include \
			-I$(TOP_DIR)/models/math/include \
			-I$(TOP_DIR)/models/aux/include \
			-I$(TOP_DIR)/models/sensor/include \
			-I$(TOP_DIR)/models/driver/include \
			-I$(TOP_DIR)/models/icf/include \
			-I$(TOP_DIR)/models/equipment_protocol/include \
			-I$(TOP_DIR)/models/flight_events/include
TRICK_CFLAGS += --std=gnu11 ${INCLUDES} -g -D_GNU_SOURCE -DCONFIG_SIL_ENABLE
TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
TRICK_CXXFLAGS += --std=c++17 ${INCLUDES} -g -DCONFIG_SIL_ENABLE
TRICK_CXXFLAGS += -Wall -Wextra -Wshadow
TRICK_USER_LINK_LIBS += -larmadillo -lhiredis
MAKEFLAGS += -j16