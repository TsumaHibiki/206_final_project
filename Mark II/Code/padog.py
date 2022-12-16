
exec(open('config_s.py').read())  


import PA_SERVO
import PA_GAIT
import PA_IK
import PA_ATTITUDE
import PA_STABLIZE
import time
from math import sin,cos,pi,atan,tan,floor
from machine import Pin,PWM,ADC,Timer,time_pulse_us,UART

adc = ADC(Pin(33))
pin_servo_vol = Pin(25,Pin.OUT)

#======UART=======
uart6=UART(2,115200,tx=26,rx=27)
uart_per_add=0
def UART_Run():
  global uart_per_add
  uart_per_add=uart_per_add+5
  if uart_per_add>=50:
    uart_per_add=0
    command=""
    if uart6.any():
      read = uart6.read(1).decode('gbk')
      while read != '/':
        command = command + read
        read = uart6.read(1).decode('gbk')
      if(command != "1/" ) and command!="":
        try:
          exec(command)
          #print("exec:",command)
        except:
          pass
          #print("execerr:",command)
      command = ""
#======UART=======

#======REMOTER======
def fb_curve(x):      
  #p1 =     -0.006
  #p2 =       9
  #return p1*x + p2
  p1 =   5.334e-06
  p2 =    -0.02423
  p3 =       23.92
  return p1*x*x + p2*x + p3
  
  
remote_per_add=0
micros1_last=0
micros2_last=0
micros4_count_1=0
micros4_count_2=0
micros4_node_1=0
micros4_node_2=0
micros6_count_1=0
micros6_count_2=0
micros6_node_1=0
micros6_node_2=0


def remote_run():
  global remote_per_add,micros1_last,micros2_last
  global micros4_count_1,micros4_count_2,micros4_node_1,micros4_node_2
  global micros6_count_1,micros6_count_2,micros6_node_1,micros6_node_2
  
  remote_per_add=remote_per_add+5
  if remote_per_add>=90:
    remote_per_add=0
    micros1 = time_pulse_us(Pin(32,Pin.IN), 1,50000) 
    micros2 = time_pulse_us(Pin(26,Pin.IN), 1,50000)  
    micros6 = time_pulse_us(Pin(27,Pin.IN), 1,50000) 
    micros4 = time_pulse_us(Pin(14,Pin.IN), 1,50000)  
   
    if micros4>1500:
      micros4_count_2=0
      micros4_count_1=micros4_count_1+1
    else:
      micros4_count_1=0
      micros4_count_2=micros4_count_2+1
    
    if micros4_count_1>=3:
      micros4_node_1=1
      micros4_node_2=0
      micros4_count_1=0
    if micros4_count_2>=3:
      micros4_node_1=0
      micros4_node_2=1
      micros4_count_1=0

    if micros6>1500:
      micros6_count_2=0
      micros6_count_1=micros6_count_1+1
    else:
      micros6_count_1=0
      micros6_count_2=micros6_count_2+1
    
    if micros6_count_1>=3:
      micros6_node_1=1
      micros6_node_2=0
      micros6_count_1=0
    if micros6_count_2>=3:
      micros6_node_1=0
      micros6_node_2=1
      micros6_count_1=0 
    if micros6_node_1==1 and (micros1+micros2+micros6+micros4)>1000:
      stable(True)
    else:
      stable(False)

    if floor(fb_curve(micros2))>=6:
      thr=6
    elif floor(fb_curve(micros2))<=-3:
      thr=-3
    else:
      thr=floor(fb_curve(micros2))


    if micros4_node_1==1 and (micros1+micros2+micros6+micros4)>1000:
      if micros1<=(1500-200) and abs(micros1_last-micros1)<100:    
        move(2.5,-1,1)
      elif micros1>=(1500+200) and abs(micros2_last-micros2)<100:   
        move(2.5,1,-1)
      else:
        if abs(micros2_last-micros2)<50:
          move(thr,1,1)
    else:
      move(0,0,0)
      
    
    micros1_last=micros1
    micros2_last=micros2
    
    
    
#======REMOTER======


alarm_pin = PWM(Pin(13),freq=0, duty=0)


alarm_flash_node=0
sound_freq1=0
sound_freq2=0
alarm_per=0
alarm_per_add=0
alarm_time_per=0
loop_speed_mode=0  
loop_speed_mode_sc=0

def alarm_run():
  global alarm_flash_node,alarm_per_add
  alarm_per_add=alarm_per_add+5
  if alarm_per_add>=alarm_time_per:
    alarm_per_add=0
    if alarm_flash_node==0:
      alarm_pin.freq(sound_freq1)
      alarm_flash_node=1
    else:
      alarm_flash_node=0
      alarm_pin.freq(sound_freq2)

def alarm(alarm_per,s_freq,s_freq2):
  global sound_freq1,sound_freq2,alarm_time_per
  sound_freq1=s_freq;sound_freq2=s_freq2
  if alarm_per==0:
    alarm_pin.freq(0)
    alarm_pin.duty(0)
    alarm_time_per=0
  else:
    alarm_time_per=alarm_per
    alarm_pin.duty(500)
    
def start_ring():
  alarm(10,100,100)
  time.sleep(0.15)
  alarm(10,300,300)
  time.sleep(0.15)
  alarm(10,400,400)
  time.sleep(0.15)
  alarm(10,600,600)
  time.sleep(0.15)
  alarm(10,800,800)
  time.sleep(0.15)
  alarm(0,0,0)
  time.sleep(1)



selfadd=0
def do_connect_AP():
  global selfadd
  import network 
  wifi = network.WLAN(network.AP_IF) 
  wifi.config(essid='MARK II')
  if not wifi.isconnected(): 
    print('Wait for MARK II')
    print('Wait cellphone to connect')
    alarm(300,460,0)
    wifi.active(True) 
    while not wifi.isconnected():
      pass
  selfadd=wifi.ifconfig()[0]
  print("connect successfully")
  alarm(10,660,460)
  time.sleep(0.5)
  alarm(0,0,0)


t=0
init_x=0;init_y=-110
ges_x_1=0;ges_x_2=0;ges_x_3=0;ges_x_4=0
ges_y_1=init_y;ges_y_2=init_y;ges_y_3=init_y;ges_y_4=init_y
x1=0;x2=0;x3=0;x4=0;y1=0;y2=0;y3=0;y4=0;
PIT_S=0;ROL_S=0;X_S=0;PIT_goal=0;ROL_goal=0;X_goal=0
spd=0;spd_goal=0;L=0;R=0
R_H=abs(init_y);H_goal=110
init_case=0
key_stab=False;gait_mode=0
stop_run_node=0
speed_init=speed
acc_z=0
IK_ERROR=0;normal_node=0;error_node=0;empty_power_count=0
pit_max_ang=25   
rol_max_ang=20  
Kp_V=1         
act_tran_mov_kp=tran_mov_kp
def mechan_offset_corr(x):
  p1=0.006649
  p2=0.4414
  p3=5.53
  return p1*x*x+p2*x+p3

def read_voltage(x):  
  p1 = 0.002575
  p2 = 0.7446
  return p1*x + p2
  
def servo_output(case,init,ham1,ham2,ham3,ham4,shank1,shank2,shank3,shank4):
  if case==0 and init==0:

    PA_SERVO.angle(2, init_1h+90-ham1) 
    PA_SERVO.angle(3, (init_1s-90)+mechan_offset_corr(shank1)) 

    PA_SERVO.angle(13, init_2h-90+ham2)  
    PA_SERVO.angle(12, (init_2s+90)-mechan_offset_corr(shank2))
  
    PA_SERVO.angle(10, init_3h-90+ham3) 
    PA_SERVO.angle(11, (init_3s+90)-mechan_offset_corr(shank3)) 

    PA_SERVO.angle(5, init_4h+90-ham4) 
    PA_SERVO.angle(4, (init_4s-90)+mechan_offset_corr(shank4)) 
  else:
    
    PA_SERVO.angle(2, init_1h)  
    PA_SERVO.angle(3, init_1s)  
  
    PA_SERVO.angle(13, init_2h)  
    PA_SERVO.angle(12, init_2s)  
    
    PA_SERVO.angle(10, init_3h)  
    PA_SERVO.angle(11, init_3s)  
    
    PA_SERVO.angle(5, init_4h)  
    PA_SERVO.angle(4, init_4s)  


def height(goal):    
    global H_goal
    H_goal=goal

def gesture(PIT,ROL,X):
    global PIT_goal,ROL_goal,X_goal
    PIT_goal=PIT
    ROL_goal=ROL
    X_goal=X


def g(PIT):
    global PIT_goal
    PIT_goal=PIT
    
def m(spd_,L_,R_):
    global spd,L,R
    spd=spd_;L=L_;R=R_
    
    


def move(spd_,L_,R_):
    global spd_goal,L,R
    spd_goal=spd_;L=L_;R=R_
  
def stable(key):
    global key_stab,speed
    key_stab=key
    if key==True:
      speed=speed_init+0.01   
    else:
      speed=speed_init   
  
def servo_init(key):
    global init_case
    init_case=key
    
def gait(mode):   
    global gait_mode
    gait_mode=mode
  

def swing_curve_generate(t,Tf,xt,zh,x0,z0,xv0):

  # X Generator
  if t>=0 and t<Tf/4:
    xf=(-4*xv0/Tf)*t*t+xv0*t+x0
    
  if t>=Tf/4 and t<(3*Tf)/4:
    xf=((-4*Tf*xv0-16*xt+16*x0)*t*t*t)/(Tf*Tf*Tf)+((7*Tf*xv0+24*xt-24*x0)*t*t)/(Tf*Tf)+((-15*Tf*xv0-36*xt+36*x0)*t)/(4*Tf)+(9*Tf*xv0+16*xt)/(16)
    
  if t>(3*Tf)/4:
    xf=xt

  # Z Generator
  if t>=0 and t<Tf/2:
    zf=(16*z0-16*zh)*t*t*t/(Tf*Tf*Tf)+(12*zh-12*z0)*t*t/(Tf*Tf)+z0
  
  if t>=Tf/2:
    zf=(4*z0-4*zh)*t*t/(Tf*Tf)-(4*z0-4*zh)*t/(Tf)+z0
      
  #Record touch down position
  x_past=xf
  t_past=t
  
  # # Avoid zf to go zero
  if zf<=0:
    zf=0
  #x,z position,x_axis stop point,t_stop point;depend on when the leg stop
  
  return xf,zf,x_past,t_past


def support_curve_generate(t,Tf,x_past,t_past,zf):
 
  # Only X Generator
  average=x_past/(1-Tf)
  xf=x_past-average*(t-t_past)
  return xf,zf
  
def alarm_and_servo_control():
  global normal_node,error_node,empty_power_count
  judge_num_node=0
  #Battary judge
  if read_voltage(adc.read())<5.5 and read_voltage(adc.read())>0.8:  
    empty_power_count=empty_power_count+1
  else:
    empty_power_count=0
  if empty_power_count>=200:
    judge_num_node=judge_num_node+1
    empty_power_count=200
    alarm(200,1000,0)
  
  if read_voltage(adc.read())<0.8:  
    judge_num_node=judge_num_node+1
  if IK_ERROR==1:
    judge_num_node=judge_num_node+1
  #IK_ERROR ALARM
  if IK_ERROR==1 and error_node==0:
    alarm(300,100,0)
    error_node=1
    normal_node=0
    print("Servo stop")
  if IK_ERROR==0 and normal_node==0:   #Focus on Clear to Zero
    alarm(0,0,0)
    error_node=0
    normal_node=1
  return judge_num_node
      
def mainloop():
  global t
  global R_H
  global PIT_S,ROL_S,X_S,act_tran_mov_kp,PIT_goal,ROL_goal,spd
  global ges_x_1,ges_x_2,ges_x_3,ges_x_4
  global ges_y_1,ges_y_2,ges_y_3,ges_y_4
  global IK_ERROR
  #Servo ON/OFF CONTROL
  if alarm_and_servo_control()==0:
    pin_servo_vol.value(1)
  else:
    PA_SERVO.release()
    pin_servo_vol.value(0)

  if stop_run_node==1:
    return 0

  if gait_mode==0:
    act_tran_mov_kp=tran_mov_kp
    if t>=1:
      t=0
    elif L==0 and R==0:
      t=0
    else:
     t=t+speed
    P_=PA_GAIT.trot(t,spd*10,h,L,L,R,R)
  elif gait_mode==1:
    act_tran_mov_kp=tran_mov_kp/2   #Walk Gait Slow down X_S change
    P_=PA_GAIT.walk(t,spd*10,h,L,L,R,R)
    if t>=2.5:
      t=0
    elif L==0 and R==0:
      t=0
    else:
      pass
        

  if spd>spd_goal:
    spd=spd-abs(spd-spd_goal)*Kp_V
  elif spd<spd_goal:
    spd=spd+abs(spd-spd_goal)*Kp_V

  if R_H>H_goal:
    R_H=R_H-abs(R_H-H_goal)*Kp_H
  elif R_H<H_goal:
    R_H=R_H+abs(R_H-H_goal)*Kp_H
  
  if key_stab!=True:     
    if PIT_S>PIT_goal:  
      PIT_S=PIT_S-abs(PIT_S-PIT_goal)*pit_Kp_G
    elif PIT_S<PIT_goal:
      PIT_S=PIT_S+abs(PIT_S-PIT_goal)*pit_Kp_G

    if ROL_S>ROL_goal:  
      ROL_S=ROL_S-abs(ROL_S-ROL_goal)*rol_Kp_G
    elif ROL_S<ROL_goal:
      ROL_S=ROL_S+abs(ROL_S-ROL_goal)*rol_Kp_G
  else:
    PIT_S=PIT_goal
    ROL_S=ROL_goal
    #X_S=X_goal

  if X_S>X_goal:   
    X_S=X_S-abs(X_S-X_goal)*act_tran_mov_kp
  elif X_S<X_goal:
    X_S=X_S+abs(X_S-X_goal)*act_tran_mov_kp

  
  if PIT_S>=pit_max_ang:PIT_S=pit_max_ang
  if PIT_S<=-pit_max_ang:PIT_S=-pit_max_ang
  if ROL_S>=rol_max_ang:ROL_S=rol_max_ang
  if ROL_S<=-rol_max_ang:ROL_S=-rol_max_ang

  
  if gait_mode==0:
    if spd>=0 and (L+R)!=0:
      P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S+abs(spd)*trot_cg_f,R_H)
    elif spd<0 and (L+R)!=0:
      P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S-abs(spd)*trot_cg_b,R_H)
    elif (L+R)==0:    
      P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S+abs(spd)*trot_cg_t,R_H)
  else:
    P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S,R_H)
  ges_x_1=P_G[0];ges_x_2=P_G[1]; ges_x_3=P_G[2]; ges_x_4=P_G[3];ges_y_1=P_G[4];ges_y_2=P_G[5]; ges_y_3=P_G[6]; ges_y_4=P_G[7]
 
  #
  if key_stab==True:
    if abs(0-spd)<=0.1 and L==0 and R==0:
      PA_STABLIZE.stab()
    else:  
      PIT_goal=0
      ROL_goal=0

  try:
    A_=PA_IK.ik(ma_case,l1,l2,P_[0]+ges_x_1,P_[1]+ges_x_2,P_[2]+ges_x_3,P_[3]+ges_x_4,P_[4]+ges_y_1,P_[5]+ges_y_2,P_[6]+ges_y_3,P_[7]+ges_y_4)
    servo_output(ma_case,init_case,A_[0],A_[1],A_[2],A_[3],A_[4],A_[5],A_[6],A_[7])
    IK_ERROR=0
  except:
    IK_ERROR=1














