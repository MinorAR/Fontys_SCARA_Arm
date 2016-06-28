CON
'Set 80Mhz
  _clkmode=xtal1+pll16x
  _xinfreq = 5000000

' ID
  ID = "3"

' Serial port
  TXPIN = 6
  RXPIN = 7 
  CR = 13
  LF = 10
  TXD = 31
  RXD = 30
  Baud = 9600

'MAE3 PWM absolute encoder
  MAE0Pin  = 16            

VAR
'MAE encoder
  Long MAEPos, MAETime
  Word MAECntr
  Long MAEStack[100]
  Long MAECog
  
OBJ
  Mae          : "MAE3"         
  Ser           : "Parallax Serial Terminal"  'FullDuplex"
  Ser2          : "Parallax Serial Terminal" 'FullDuplex"
  t             : "Timing"
  num           : "Simple_Numbers"
  
PUB Main | ch
  Init

  repeat
    ch := Ser.RxCheck

    if (ch <> -1) AND (ch == ID)
      Ser.dec(MAEPos)
      Ser.tx(LF)

      Ser2.dec(MAEPos)
      Ser2.tx(CR)
      Ser2.tx(LF)

      {Ser.str(string("I got pinged: "))
      Ser.tx(ID)
      Ser.tx(LF)

      Ser2.str(string("I got pinged: "))
      Ser2.tx(ID)
      Ser2.tx(LF)}
      
PRI MAESense | tt1 , lMAE0Pin, lMAE1Pin, lMAE2Pin, lMAE3Pin
  lMAE0Pin:= MAE0Pin
  
  Repeat
    tt1:=cnt
    MAEPos:= MAE.Pos(lMAE0Pin)
    MAECntr++                   'Update alive counter
    MAETime:=(cnt - tt1)/80000      'Cycle time in ms
    
PRI Init | ii
  Ser2.start(Baud)    '' Initialize serial communication to the PC through the USB connector
  Ser.startRxTx(RXPIN, TXPIN, 0, Baud)
  
  t.pause1ms(1000)
  Ser.clear
  Ser2.clear 

  MAECog:=CogNew(MAESense, @MAEStack)                   'Start MAE sensing
  t.pause1ms(100)