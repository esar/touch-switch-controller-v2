
typedef enum _RakoMsgType
{
	RAKO_MSG_TYPE_COMMAND  = 0,
	RAKO_MSG_TYPE_DATA     = 2

} RakoMsgType;

typedef enum _RakoCmd
{
	RAKO_CMD_OFF          = 0x0,
	RAKO_CMD_RAISE        = 0x1,
	RAKO_CMD_LOWER        = 0x2,
	RAKO_CMD_SCENE1       = 0x3,
	RAKO_CMD_SCENE2       = 0x4,
	RAKO_CMD_SCENE3       = 0x5,
	RAKO_CMD_SCENE4       = 0x6,
	RAKO_CMD_PROGRAM_MODE = 0x7,
	RAKO_CMD_IDENT        = 0x8,
	RAKO_CMD_LOW_BATTERY  = 0xa,
	RAKO_CMD_EEPROM_WRITE = 0xb,
	RAKO_CMD_LEVEL_SET    = 0xc,
	RAKO_CMD_STORE        = 0xd,
	RAKO_CMD_STOP         = 0xf,
	
} RakoCmd;

// NOTE: Field order in the RakoMsg bit fields is reversed due 
//       to the uint32_t type being stored in little-endian order.
typedef union _RakoMsg RakoMsg;
union _RakoMsg
{
	struct
	{
		uint32_t data : 28;
		uint32_t type : 4;

	} unknown;
  
	struct
	{
		uint32_t padding : 3;

		uint32_t check   : 1;
		uint32_t command : 4;
		uint32_t channel : 4;
		uint32_t room    : 8;
		uint32_t house   : 8;
		uint32_t type    : 4;

	} command;

	struct
	{
		uint32_t padding : 3;

		uint32_t check   : 1;
		uint32_t address : 8;
		uint32_t data    : 8;
		uint32_t house   : 8;
		uint32_t type    : 4;

	} data;

	uint32_t raw;

} __attribute__((packed));

