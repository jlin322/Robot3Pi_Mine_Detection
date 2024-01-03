#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/io.h>
#include "lcd_driver.h"
#include "port_macros.h"

#define Sensor0 0
#define Sensor1 1
#define Sensor2 2
#define Sensor3 3
#define Sensor4 4
#define LEFTBUTTON 1
#define MIDBUTTON 4
#define RIGHTBUTTON 5
#define RIGHTMOTOR3 3
#define LEFTMOTOR5 5
#define LEFTMOTOR6 6

#define INCREMENT 25
#define PWM_TOP 100
#define TURN_TIME 416  //turn time for right side
#define TURN_TIMEL 416
#define duty_cycle1 20
#define duty_cycle2 20
int INCH = 600;

int sensor0 = 0;
int sensor1 = 0;
int sensor2 = 0;
int sensor3 = 0;
int sensor4 = 0;
char sensorval[6]={'b','b','b','b','b','\0'};
int findMines = 0;
int minesFound = 0;
char findMinesChar[1];          //displays x value on lcd
int mine_found_or_not = 0;
char mchar[2];
int distance=0;
int second = 0;

//-------------------------------------------Basic Voids-------------------------------------------//
void brake(){
  PORTD |= (1<<LEFTMOTOR5)|(1<<LEFTMOTOR6)|(1<<RIGHTMOTOR3);
  PORTB |= (1<<RIGHTMOTOR3);
}
void forwardL(){
  PORTD &= ~(1<<LEFTMOTOR5);
  PORTD |= (1<<LEFTMOTOR6);
}
void forwardR(){
  PORTD &= ~((1<<RIGHTMOTOR3));
  PORTB |= (1<<RIGHTMOTOR3);
}
void straightLineForward(){
  unsigned int pwm_counter=0;
  for(int i = 0; i<=INCH; i++){                          //moves forward an inch
    pwm_counter = pwm_counter + 1;
    if( pwm_counter >= PWM_TOP ){
      pwm_counter = 0;
    }
    if( pwm_counter < duty_cycle1 ){
      forwardL();
    }
    else{
      brake();
    }

    if( pwm_counter < duty_cycle2){
      forwardR();
    }
    else{
      brake();
    }
  }
  brake();
}
void backwardL(){
  PORTD |= ((1<<LEFTMOTOR5));
  PORTD &= ~(1<<LEFTMOTOR6);
}
void backwardR(){
  PORTB &= ~((1<<RIGHTMOTOR3));
  PORTD |= (1<<RIGHTMOTOR3);
}
void straightLineBackward(){
  unsigned int pwm_counter=0;
  for(int i = 0; i<=INCH; i++){                          //moves forward an inch
    pwm_counter = pwm_counter + 1;
    if( pwm_counter >= PWM_TOP ){
      pwm_counter = 0;
    }
    if( pwm_counter < duty_cycle1 ){
      backwardL();
    }
    else{
      brake();
    }


    if( pwm_counter < duty_cycle2){
      backwardR();
    }
    else{
      brake();
    }
  }
  brake();
}

void turn_right(){
  unsigned int pwm_counter=0;
  for(int i = 0; i<=TURN_TIME; i++){
    pwm_counter = pwm_counter + 1;
    if( pwm_counter >= PWM_TOP ){
      pwm_counter = 0;
    }
    if( pwm_counter < duty_cycle1 ){
        PORTD &= ~(1<<LEFTMOTOR5);
        PORTD |= (1<<LEFTMOTOR6);
    }
    else{
    brake();
    }

    if( pwm_counter < duty_cycle2){
      PORTD |= (1<<RIGHTMOTOR3);
      PORTB &= ~(1<<RIGHTMOTOR3);
    }
    else{
      brake();
    }
  }
}
void turn_left(){
  unsigned int pwm_counter=0;
  for(int i = 0; i<=TURN_TIMEL; i++){
    pwm_counter = pwm_counter + 1;
    if( pwm_counter >= PWM_TOP ){
      pwm_counter = 0;
    }
    if( pwm_counter < duty_cycle1 ){
      PORTD &= ~(1<<LEFTMOTOR6);
      PORTD |= (1<<LEFTMOTOR5);
    }
    else{
      brake();
    }
    if( pwm_counter < duty_cycle2){
      PORTB |= (1<<RIGHTMOTOR3);
      PORTD &= ~(1<<RIGHTMOTOR3);
    }
    else{
      brake();
    }
  }
}

//---------------------------LCD_Sensor_Values------------------------------//
void LCDSensor(){
    if((PINC & (1<<Sensor0)) == 0){  //check if sensor0 is on
      //display first value as 1
      sensorval[0] = 'w';
      sensor0 = 1;
    }
    else{
      sensorval[0] = 'b';
      sensor0 = 0;
    }
    if((PINC & (1<<Sensor1)) == 0){  //check if sensor1 is on
      //display first value as 1
      sensorval[1] = 'w';
      sensor1 = 1;
    }
    else{
      sensorval[1] = 'b';
      sensor1 = 0;
    }
    if((PINC & (1<<Sensor2)) == 0){  //check if sensor2 is on
      //display first value as 1
      sensorval[2] = 'w';
      sensor2 = 1;
    }
    else{
      sensorval[2] = 'b';
      sensor2 = 0;
    }
     if((PINC & (1<<Sensor3)) == 0){  //check if sensor3 is on
      //display first value as 1
      sensorval[3] = 'w';
      sensor3 = 1;
    }
    else{
      sensorval[3] = 'b';
      sensor3 = 0;
    }
     if((PINC & (1<<Sensor4)) == 0){  //check if sensor4 is on
      //display first value as 1
      sensorval[4] = 'w';
      sensor4 = 1;
     }
    else{
      sensorval[4] = 'b';
      sensor4 = 0;
    }
    //LCD_execute_command(CLEAR_DISPLAY);
    LCD_move_cursor_to_col_row(1, 3);
    LCD_print_String(sensorval);



}

//---------------------------Mine Deectecting------------------------------//
void MineDetection(){
  if((sensor0==1)&&(sensor1==1)&&(sensor2==1)&&(sensor3==1)&&(sensor4==1)){
    brake();
    displayMineFound();
    INCH = 300;
      for(int r =0; r<=INCH; r++){
      //all sensors detect black
      straightLineBackward();
      LCDSensor();
      brake();

    //  _delay_ms(2000);
    }
    for(int r =0; r<=(4*TURN_TIME); r++){
        turn_left();

  }
  brake();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==1)&&(sensor4==0)){
    brake();
    INCH = 300;
      for(int r =0; r<=INCH; r++){
      //all sensors detect black
      straightLineBackward();
      LCDSensor();
      brake();
    //  _delay_ms(2000);
    }
    for(int r =0; r<=(4*TURN_TIME); r++){
        turn_left();
  }
  brake();
  }
}
void displayMineFound(){
  minesFound += 1;            //increment # mines found and display in LCD
  LCD_move_cursor_to_col_row(6,0);
  itoa(minesFound, mchar,10);
  LCD_print_String(mchar);
}

//-------------------------------------------Detecting Path-------------------------------------------//
void directionTurning(){
  if(second == 0  ){
    if(distance % 2 == 0){      //if even, turn left
      for(int i = 0; i<=TURN_TIMEL;i++){
        turn_left();
      }
    }
    else if(distance % 2 == 1){
      for(int i = 0; i<=TURN_TIME;i++){
        turn_right();
      }
    }
    second = 1;
  }
  else if(second == 1){
    if(distance % 2 == 0){      //if even, turn left
      for(int i = 0; i<=TURN_TIMEL;i++){
        turn_left();
      }
    }
    else if(distance % 2 == 1){
      for(int i = 0; i<=TURN_TIME;i++){
        turn_right();
      }
    }
    second = 0;
    distance += 1;
  }
  /*
  char xchar[1];

  itoa(distance,xchar,10);
  LCD_print_String(xchar);*/
}

void FindCorner(){              //finds the upper left corner
  brake();
  LCDSensor();

  if((sensor0==1)&&(sensor2==0)&&(sensor4==1)){           //Straight line
    straightLineForward();
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==1)){     //autocorrection since veers left
    for(int r =0; r<=30; r++){
      turn_right();
    }
  }
  if((sensor0==1)&&(sensor1==0)&&(sensor2==1)&&(sensor3==1)&&(sensor4==1)){    //veers right
    for(int r =0; r<=30; r++){
      turn_left();
    }
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==1)){
    straightLineForward();
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){       // All sensors detect black
   straightLineForward();
   LCDSensor();
  }

  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==1)&&(sensor4==0)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==1)&&(sensor4==1)){      // Corner Detected
    LOOP();
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==1)&&(sensor4==1)){      // Corner Detected
   LOOP();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==1)&&(sensor3==1)&&(sensor4==1)){      // Corner Detected
    LOOP();
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){      // Corner Detected
    LOOP();
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==0)){      // Corner Detected
    LOOP();
    LCDSensor();
  }
  if((sensor1 == 0)||(sensor3 == 0)){
    straightLineForward();
  }
   if((sensor0==0)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==1)){
    straightLineForward();
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==0)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==1)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==1)&&(sensor4==0)){  // Turn left
    brake();
    INCH = 302;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
    }
    for(int r =0; r<=TURN_TIME; r++){
      turn_left();
    }
    LCDSensor();
  }

  MineDetection();
  if(minesFound >= findMines){      //stop when # of mines are found
    brake();
  }
}

void LOOP(){
  brake();
  LCDSensor();

  if((sensor0==1)&&(sensor2==0)&&(sensor4==1)){           //Straight
    straightLineForward();
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==1)){    //veers right
    straightLineForward();
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==1)){     //autocorrection since veers left
    for(int r =0; r<=40; r++){
      turn_right();
    }
  }
  if((sensor0==1)&&(sensor1==0)&&(sensor2==1)&&(sensor3==1)&&(sensor4==1)){    //veers right
    for(int r =0; r<=40; r++){
      turn_left();
    }
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){       // All Black
   straightLineForward();
   LCDSensor();
  }
 if((sensor0==1)&&(sensor1==0)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){
  for(int r =0; r<=50; r++){
    turn_right();
    }
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==0)&&(sensor4==1)){
   for(int r =0; r<=50; r++){
     turn_left();
    }
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==1)&&(sensor4==0)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==0)&&(sensor3==1)&&(sensor4==1)){      // Corner
   brake();
   INCH = 306;
   for(int r =0; r<=INCH; r++){
     straightLineForward();
     LCDSensor();
     brake();
    }
    for(int r =0; r<=TURN_TIME; r++){
      turn_left();
    }
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==0)&&(sensor2==1)&&(sensor3==1)&&(sensor4==1)){      // Corner
    brake();
    INCH = 306;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
    }
    for(int r =0; r<=TURN_TIME; r++){
      turn_left();
    }
    LCDSensor();
  }
  if((sensor1 == 0)||(sensor3 == 0)){
    straightLineForward();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==1)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){
    straightLineForward();
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==0)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==1)){
   straightLineForward();
   LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==1)&&(sensor4==0)){  // Turn left
    brake();
    INCH = 300;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
    }
    int randd = rand();
    if(randd % 2 == 0){
      for(int r =0; r<=TURN_TIME; r++){
          turn_right();
      }
    }
    else{
      for(int r =0; r<=TURN_TIMEL; r++){
          turn_left();
        }
    }
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){  // Turn left
    brake();
    INCH = 305;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
      }
    for(int r =0; r<=TURN_TIME; r++){
      turn_right();
      }
    LCDSensor();
  }
  if((sensor0==0)&&(sensor1==1)&&(sensor2==1)&&(sensor3==1)&&(sensor4==1)){  // Turn left
    brake();
    INCH = 305;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
    }
    for(int r =0; r<=TURN_TIME; r++){
      turn_right();
    }
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==1)&&(sensor3==0)&&(sensor4==0)){  // Turn left
    brake();
    INCH = 304;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
    }
    for(int r =0; r<=TURN_TIME; r++){
      turn_left();
    }
    LCDSensor();
  }
  if((sensor0==1)&&(sensor1==1)&&(sensor2==0)&&(sensor3==0)&&(sensor4==0)){
    brake();
    INCH = 303;
    for(int r =0; r<=INCH; r++){
      straightLineForward();
      LCDSensor();
      brake();
    }
    directionTurning();
    LCDSensor();
  }
  MineDetection();
}

int main() {
  unsigned int last_mid_button_state = (PINB & (1<<MIDBUTTON));
  unsigned int mid_button_pressed=0;
  unsigned int last_left_button_state = (PINB & (1<<LEFTBUTTON));
  unsigned int left_button_pressed=0;
  unsigned int last_right_button_state = (PINB & (1<<RIGHTBUTTON));
  unsigned int right_button_pressed=0;

  DDRB &= ~((1<<RIGHTBUTTON)|(1<<LEFTBUTTON)|(1<<MIDBUTTON)) ; //sets to INPUT (PB1,PB4,PB5,PB0)
  DDRB |= (1<<RIGHTMOTOR3); //sets motor to OUTPUT (PB3,PB4,PB5,PB1)
  DDRD |= (1<<LEFTMOTOR5)|(1<<LEFTMOTOR6)|(1<<RIGHTMOTOR3);//outputs (PD5,PD6,PD3,PD7)
  DDRC &= ~((1<<Sensor0)|(1<<Sensor1)|(1<<Sensor2)|(1<<Sensor3)|(1<<Sensor4)) ; //Sensors are inputs

  PINC |= ((1<<Sensor0)|(1<<Sensor1)|(1<<Sensor2)|(1<<Sensor3)|(1<<Sensor4));
  PINB &= ~((1<<LEFTBUTTON)|(1<<RIGHTBUTTON)|(1<<MIDBUTTON));
  brake();

  initialize_LCD_driver();
  LCD_execute_command(TURN_ON_DISPLAY);
  LCD_execute_command(CLEAR_DISPLAY);

  LCD_move_cursor_to_col_row(0, 4);     //# mines found
  itoa(minesFound, mchar,10);
  LCD_print_String(mchar);
  _delay_ms(3000);

  while(1){
    if( (PINB & (1<<1)) != last_left_button_state ){
      if( (PINB & (1<<1)) == 0 ){
        left_button_pressed=1;
      }
      last_left_button_state = (PINB & (1<<1));
    }
    else{
      left_button_pressed=0;
    }
    _delay_us(300);
    if(left_button_pressed == 1){
      if(findMines >= 0){
        findMines = findMines-1;
        itoa(findMines,findMinesChar,10);
        LCD_move_cursor_to_col_row(0, 0);
        LCD_execute_command(CLEAR_DISPLAY);
        LCD_print_String(findMinesChar);
      }
    }
    if( (PINB & (1<<5)) != last_right_button_state ){
      if( (PINB & (1<<5)) == 0 ){
        right_button_pressed=1;
      }
      last_right_button_state = (PINB & (1<<5));
    }
    else{
      right_button_pressed=0;
    }
    _delay_us(300);
    if(right_button_pressed == 1){
      findMines = findMines+1;
      itoa(findMines,findMinesChar,10);
      LCD_move_cursor_to_col_row(0, 0);
      LCD_execute_command(CLEAR_DISPLAY);
      LCD_print_String(findMinesChar);
    }
    if( (PINB & (1<<MIDBUTTON)) != last_mid_button_state ){
      if( (PINB & (1<<MIDBUTTON)) == 0 ){
        mid_button_pressed=1;
      }
      last_mid_button_state = (PINB & (1<<MIDBUTTON));
    }
    else{
      mid_button_pressed=0;
    }
    if(mid_button_pressed == 1){         //ROBOT MOVEMENT HAPPENS HERE

      LCD_move_cursor_to_col_row(6,0);     //# mines found
      itoa(minesFound, mchar,10);
      LCD_print_String(mchar);
      while(1){
        LCD_move_cursor_to_col_row(1, 3);
        LCD_print_String(sensorval);
        LCDSensor();

        FindCorner();
        if(minesFound >= findMines){
          break;
        }
        brake();
        _delay_us(10);
      }
    }
  }
  return 0;
}
