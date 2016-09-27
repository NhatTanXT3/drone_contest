

#ifndef QUADROTOR_CURRENT_WORK_SERIAL_H_
#define QUADROTOR_CURRENT_WORK_SERIAL_H_

// Define uart for hardware
#define UART_BASE_USE 			UART0_BASE // define uart module used in this program for communication
//#define UART_BASE_KINECT	UART6_BASE
#define UART_COM_2_CONTROLLER_				UART0_BASE
#define UART_IMAGE_SENSOR_2_CONTROLLER_		UART6_BASE



#define TERMINATOR_ '\0'

#define CN_1_ '~'
#define CN_2_ '!'
#define CN_3_ '@'
#define CN_4_ '#'
#define CN_5_ '$'
#define CN_6_ '%'
#define CN_7_ '^'
#define CN_8_ '*'
#define CN_9_ '('
#define CN_10_ ')'


typedef struct {
	volatile unsigned char Flag_receive;
	 char Command_Data[40];
//	 char TX_software_FIFO[100];
} MyUart;
extern MyUart Uart;
extern MyUart UartKinect;
/*
 * Define the protocol
 */
#define COM2CTL_RUN					42//'*'
#define COM2CTL_RUN_ON					49//'1'
#define COM2CTL_RUN_OFF					48//'0'

#define COM2CTL_TEST_MOTOR			33//'!'
#define COM2CTL_TEST_MOTOR_ON			49//'1'
#define COM2CTL_TEST_MOTOR_OFF			48//'0'
#define COM2CTL_TEST_MOTOR_1		34//'"'
#define COM2CTL_TEST_MOTOR_2		35//'#'
#define COM2CTL_TEST_MOTOR_3		36//'$'
#define COM2CTL_TEST_MOTOR_4		37//'%'

#define COM2CTL_DISPLAY				38//'&' reserve 38 to 41
#define COM2CTL_DISPLAY_ON				49//'1'
#define COM2CTL_DISPLAY_OFF				48//'0'

#define COM2CTL_MOTOR				39// '''
#define COM2CTL_MOTOR_ON				49//'1'
#define COM2CTL_MOTOR_OFF				48//'0'

#define SENSOR2CTL_POSITION			40//'('

#define COM2CTL_SET_PARAMETER		59//';'
#define COM2CTL_SET_PARAMETER_OFF		48//'0'
#define	COM2CTL_SET_PARAMETER_ROLL		60//'<' reserve 60 to 64
#define COM2CTL_SET_PARAMETER_PITCH		61//'='
#define COM2CTL_SET_PARAMETER_YAW		62//'>'
#define COM2CTL_SET_PARAMETER_Y			63//'?'
#define COM2CTL_SET_PARAMETER_X			64//'@'
#define COM2CTL_SET_PARAMETER_Z			40//'('


#define COM2CTL_SCAN_PARAMETER		60//'<'
#define COM2CTL_SET_KP				61//'='
#define COM2CTL_SET_KI				62//'>'
#define COM2CTL_SET_KD				63//'?'

#define CTL2COM_SCAN_PARAMETER		60//'<'
#define CTL2COM_DISPLAY				38//'&'

//extern char * float2num(float num);
extern void float2num(float num, char *str);
extern void int2num(int num,char *str);
extern void set_position (char *string, int16_t *position);
// init UART_port
extern void UART0_Init();
extern void UART6_Init();
extern void UART5_Init();
extern void UART1_Init();

extern void UartPutStr(uint32_t ui32Base,char *uart_str);
extern void UartGetStr(uint32_t ui32Base,char *uart_str);

extern void  UartPutStr_NonTer(uint32_t ui32Base,char *uart_str);
extern void UartPutStrLn(uint32_t ui32Base,char *uart_str);

// hien thi so thap phan dang xxx.xx
extern void UART_float_display(uint32_t ui32Base , float num,bool terminator);

// hien thi so nguyen dang chuoi, so co dang xxxxx
extern  void UART_int_display(uint32_t ui32Base,int16_t num,bool terminator);

// gui 3 so float co dang xxx.xxaxxx.xxaxxx.xx
extern  void UART_3_float_display(uint32_t ui32Base,float num1, float num2, float num3 );

// send 3 integer number xxxxxaxxxxxaxxxxx|terminator|
extern void UART_3_int_display(uint32_t ui32Base,int16_t num1, int16_t num2, int16_t num3 );
extern void UART_3_uint32_display(uint32_t ui32Base,uint32_t num1, uint32_t num2, uint32_t num3 );
// send n integer number
extern void UART_n_int_display(uint32_t ui32Base,uint32_t *num,uint8_t size);

// gan gia tri choi so cho bien kp
extern void set_float_value (char *string,float *kp);
extern void set_int_value (char *string,int32_t *kp);
extern void float2str(float num,char *kq);
extern void int2str(int32_t num,char *kq);

extern char *num2str (unsigned char num);

#endif //QUADROTOR_CURRENT_WORK_SERIAL_H_

//======Ascii code==========================
//	binary|demical|hexa|ascii
//	010 0000	32	20	space (sp)
//	010 0001	33	21	!
//	010 0010	34	22	"
//	010 0011	35	23	#
//	010 0100	36	24	$
//	010 0101	37	25	%
//	010 0110	38	26	&
//	010 0111	39	27	'
//	010 1000	40	28	(
//	010 1001	41	29	)
//	010 1010	42	2A	*
//	010 1011	43	2B	+
//	010 1100	44	2C	,
//	010 1101	45	2D	-
//	010 1110	46	2E	.
//	010 1111	47	2F	/
//	011 0000	48	30	0
//	011 0001	49	31	1
//	011 0010	50	32	2
//	011 0011	51	33	3
//	011 0100	52	34	4
//	011 0101	53	35	5
//	011 0110	54	36	6
//	011 0111	55	37	7
//	011 1000	56	38	8
//	011 1001	57	39	9
//	011 1010	58	3A	:
//	011 1011	59	3B	;
//	011 1100	60	3C	<
//	011 1101	61	3D	=
//	011 1110	62	3E	>
//	011 1111	63	3F	?
//	100 0000	64	40	@
//	100 0001	65	41	A
//	100 0010	66	42	B
//	100 0011	67	43	C
//	100 0100	68	44	D
//	100 0101	69	45	E
//	100 0110	70	46	F
//	100 0111	71	47	G
//	100 1000	72	48	H
//	100 1001	73	49	I
//	100 1010	74	4A	J
//	100 1011	75	4B	K
//	100 1100	76	4C	L
//	100 1101	77	4D	M
//	100 1110	78	4E	N
//	100 1111	79	4F	O
//	101 0000	80	50	P
//	101 0001	81	51	Q
//	101 0010	82	52	R
//	101 0011	83	53	S
//	101 0100	84	54	T
//	101 0101	85	55	U
//	101 0110	86	56	V
//	101 0111	87	57	W
//	101 1000	88	58	X
//	101 1001	89	59	Y
//	101 1010	90	5A	Z
//	101 1011	91	5B	[
//	101 1100	92	5C	\
//	101 1101	93	5D	]
//	101 1110	94	5E	^
//	101 1111	95	5F	_
//	110 0000	96	60	`
//	110 0001	97	61	a
//	110 0010	98	62	b
//	110 0011	99	63	c
//	110 0100	100	64	d
//	110 0101	101	65	e
//	110 0110	102	66	f
//	110 0111	103	67	g
//	110 1000	104	68	h
//	110 1001	105	69	i
//	110 1010	106	6A	j
//	110 1011	107	6B	k
//	110 1100	108	6C	l
//	110 1101	109	6D	m
//	110 1110	110	6E	n
//	110 1111	111	6F	o
//	111 0000	112	70	p
//	111 0001	113	71	q
//	111 0010	114	72	r
//	111 0011	115	73	s
//	111 0100	116	74	t
//	111 0101	117	75	u
//	111 0110	118	76	v
//	111 0111	119	77	w
//	111 1000	120	78	x
//	111 1001	121	79	y
//	111 1010	122	7A	z
//	111 1011	123	7B	{
//	111 1100	124	7C	|
//	111 1101	125	7D	}
//	111 1110	126	7E	~
//
