#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#define F_SCL 400000UL // SERIAL CLOCK FREQUENCY
#define MPU_ADDR 0x68 // REGISTER TO READ IMU

#define TRIG1 PB5 // FRONT US TRIGGER PIN
#define ECHO1 PD3 // FRONT US ECHO PIN
#define TRIG2 PB3 // SIDE US TRIGGER PIN
#define ECHO2 PD2 // SIDE US ECHO PIN
#define IR_PIN PC0 // IR SENSOR PIN

#define LIFT_FAN PD6 // LIFT FAN PIN
#define THRUST_FAN PD5 // THRUST FAN PIN

#define SERVO_PIN PB1 // SERVO PIN

// === PID control variables ===
#define Kp 1.5
#define Ki 0.0
#define Kd 0.3

float yaw = 0;  // Current yaw angle
float zGyr = 0; // Gyroscope Z-axis rate of rotation
float target_yaw = 0; // Target yaw for PID control
float error = 0, last_error = 0, integral = 0; // Other PID variables
volatile float us1_distance = 0, us2_distance = 0, ir_distance = 0; // Sensor distance variables

// === ADC ===
uint16_t read_adc(uint8_t channel)
{
  ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Set ADC Channel
  ADCSRA |= (1 << ADSC); // Start Conversion
  while (ADCSRA & (1 << ADSC)) // Wait for Conversion to complete
    ;
  return ADC; // Return value
}

void update_ir()
{
  uint16_t ir_adc = read_adc(0); // read on channel 0
  float voltage = (ir_adc / 1023.0) * 3.3; // Convert ADC reading to voltage (Calculation from assignment 1)
  ir_distance = 27.86 * pow(voltage, -1.15); // Convert voltage to distance in cm (Calculation from assignment 1)
}

// === Ultrasonic reading by waiting for echo pulse ===
uint32_t waitPulse(uint8_t pin, uint8_t state, uint32_t timeout) // pin on portd, wanted state (HIGH = Rising edge, LOW = Falling edge), maximum wait time
{
  uint32_t count = 0; // Set count to 0
  if (state == HIGH) // If wanted state is high, wait for low
  {
    while (!(PIND & (1 << pin))) // Wait for pin to go low
    {
      if (++count >= timeout)
        return 0;
      _delay_us(1);
    }
    count = 0;
    while (PIND & (1 << pin)) // Check how long pin is high for
    {
      if (++count >= timeout)
        return 0;
      _delay_us(1);
    }
  }
  else // If wanted state is low, wait for high
  {
    while (PIND & (1 << pin)) // wait for pin to go high
    {
      if (++count >= timeout)
        return 0;
      _delay_us(1);
    }
    count = 0;
    while (!(PIND & (1 << pin))) // start count when pin goes low and count how long it is low for
    {
      if (++count >= timeout)
        return 0;
      _delay_us(1);
    }
  }
  return count; // return the count
}

void init_sensors()
{
  DDRB |= (1 << TRIG1) | (1 << TRIG2); // set triggers as outputs
  DDRD &= ~((1 << ECHO1) | (1 << ECHO2)); // set echos as inputs
  DDRC &= ~(1 << IR_PIN); // set ir pin as input

  // ADC config
  ADMUX = (1 << REFS0); // set reference voltage
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable adc and set prescaler
}

float read_ultrasonic(uint8_t trig_pin, uint8_t echo_pin)
{
  PORTB &= ~(1 << trig_pin); // set trigger low
  _delay_us(2);
  PORTB |= (1 << trig_pin); // set high with 10 us delay
  _delay_us(10);
  PORTB &= ~(1 << trig_pin); // set back to low

  uint32_t duration = waitPulse(echo_pin, HIGH, 30000); // measure pulse length
  return (duration * 0.0343) / 2.0; // return distance (Calculation from assignment 1)
}

// === Servo PWM ===
void SERVO_Init()
{
  DDRB |= (1 << SERVO_PIN); // set servo pin as output
  TCCR1A = (1 << COM1A1) | (1 << WGM11); // fast pwm, non inverting output
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << CS10); // fast pwm, update of ocr1x at top, set prescaler
  ICR1 = 2500; // top
}

void set_servo_angle(float angle_i) // set servo angle (-90 = left, 0 = center, 90 = right)
{
  float angle = angle_i + 90; 
  if (angle < 9)
    angle = 9;
  if (angle > 171)
    angle = 171;
  uint16_t pulse_width = (angle * 2000 / 90) + 1000; // set pulse width for servo
  OCR1A = pulse_width / 8;
}

// === I2C === from hardware twi library on moodle
void I2C_Init()
{
  TWSR = 0x00;
  TWBR = ((F_CPU / F_SCL) - 16) / 2;
  TWCR = (1 << TWEN);
}

void I2C_Start()
{
  TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
}

void I2C_Stop()
{
  TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void I2C_Write(uint8_t data)
{
  TWDR = data;
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
}

uint8_t I2C_Read_ACK()
{
  TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)))
    ;
  return TWDR;
}

uint8_t I2C_Read_NACK()
{
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  return TWDR;
}

// === MPU6050 ===
void MPU6050_Init() // initialize imu
{
  I2C_Start();
  I2C_Write(MPU_ADDR << 1);
  I2C_Write(0x6B);
  I2C_Write(0x00);
  I2C_Stop();
  I2C_Start();
  I2C_Write(MPU_ADDR << 1);
  I2C_Write(0x1B);
  I2C_Write(0x00);
  I2C_Stop();
  I2C_Start();
  I2C_Write(MPU_ADDR << 1);
  I2C_Write(0x19);
  I2C_Write(0x00);
  I2C_Stop();
}

int16_t MPU_Read(uint8_t reg) // read imu
{
  I2C_Start();
  I2C_Write(MPU_ADDR << 1);
  I2C_Write(reg);
  I2C_Start();
  I2C_Write((MPU_ADDR << 1) | 1);
  int16_t data = (I2C_Read_ACK() << 8) | I2C_Read_NACK();
  I2C_Stop();
  return data;
}

void Read_MPU6050() // convert imu measurement to degrees (calculation from assignment 2)
{
  int16_t gz = MPU_Read(0x47);
  zGyr = gz / 131.0; // deg/s
}

// === PID Control ===
float compute_pid(float setpoint, float current, float dt) // PID control calculations
{
  error = current - setpoint;
  integral += error * dt;
  float derivative = (error - last_error) / dt;
  last_error = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

void start_fans() // set fans ON
{
  DDRD |= (1 << LIFT_FAN) | (1 << THRUST_FAN);
  PORTD |= (1 << LIFT_FAN) | (1 << THRUST_FAN);
}

void stop_fans() // set fans OFF
{
  PORTD &= ~(1 << LIFT_FAN);
  PORTD &= ~(1 << THRUST_FAN);
}

void turn_left() // turn left by setting new target angle towards left
{
  target_yaw += 90;
  if (target_yaw > 180)
    target_yaw -= 360;
}

void turn_right() // turn right
{
  target_yaw -= 90;
  if (target_yaw < -180)
    target_yaw += 360;
}

int main(void)
{
  // initializations
  SERVO_Init();
  I2C_Init();
  init_sensors();
  MPU6050_Init();

  start_fans();

  update_ir();

  while (ir_distance > 20) // only move while ir does not detect bar
  {

    update_ir();
    us1_distance = read_ultrasonic(TRIG1, ECHO1);
    us2_distance = read_ultrasonic(TRIG2, ECHO2);

    Read_MPU6050(); // read imu and control thrust fan using pid
    float dt = 0.1;
    yaw += zGyr * dt;
    if (yaw > 180)
      yaw -= 360;
    if (yaw < -180)
      yaw += 360;

    float correction = compute_pid(target_yaw, yaw, dt);
    set_servo_angle(correction);

    if (us1_distance < 7) // if wall is close by in front, decide a direction to turn
    {
      if (us2_distance < 20) // if side wall is close, turn left
      {
        turn_left();
      }
      else // is side wall is far, turn right
      {
        turn_right();
      }

      // Wait until turn is complete
      float diff;

      do // keep turning until hovercraft turned within 5 degrees of targeted angle
      {
        Read_MPU6050(); // correct thrust fan with pid
        yaw += zGyr * dt;
        if (yaw > 180)
          yaw -= 360;
        if (yaw < -180)
          yaw += 360;

        diff = yaw - target_yaw;
        if (diff > 180)
          diff -= 360;
        if (diff < -180)
          diff += 360;

        float correction = compute_pid(target_yaw, yaw, dt);
        set_servo_angle(correction);

        update_ir();

        if (ir_distance <= 20) // check for stop bar while turning
        {
          break;
        }

        _delay_ms(100);
      } while (fabs(diff) > 5); 

      // reset pid after making a turn to align with new direction
      integral = 0;
      last_error = 0;
      target_yaw = yaw;
    }

    _delay_ms(100);
  }

  stop_fans();
  set_servo_angle(0);
}
