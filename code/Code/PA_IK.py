from math import asin,acos,atan,pi,sqrt

def ik(case,l1,l2,x1,x2,x3,x4,y1,y2,y3,y4):
    if case==0:
   
      #Leg1
      x1=-x1
      shank1=pi-acos((x1*x1+y1*y1-l1*l1-l2*l2)/(-2*l1*l2))
      fai1=acos((l1*l1+x1*x1+y1*y1-l2*l2)/(2*l1*sqrt(x1*x1+y1*y1)))
      if x1>0:
          ham1=abs(atan(y1/x1))-fai1
      elif x1<0:
          ham1=pi-abs(atan(y1/x1))-fai1
      else:
          ham1=pi-1.5707-fai1
      shank1=180*shank1/pi
      ham1=180*ham1/pi

      #Leg2
      x2=-x2
      shank2=pi-acos((x2*x2+y2*y2-l1*l1-l2*l2)/(-2*l1*l2))

      fai2=acos((l1*l1+x2*x2+y2*y2-l2*l2)/(2*l1*sqrt(x2*x2+y2*y2)))
      if x2>0:
          ham2=abs(atan(y2/x2))-fai2
      elif x2<0:
          ham2=pi-abs(atan(y2/x2))-fai2
      else:
          ham2=pi-1.5707-fai2
      shank2=180*shank2/pi
      ham2=180*ham2/pi

      #Leg3
      x3=-x3
      shank3=pi-acos((x3*x3+y3*y3-l1*l1-l2*l2)/(-2*l1*l2))
      fail3=acos((l1*l1+x3*x3+y3*y3-l2*l2)/(2*l1*sqrt(x3*x3+y3*y3)))
      if x3>0:
          ham3=abs(atan(y3/x3))-fail3
      elif x3<0:
          ham3=pi-abs(atan(y3/x3))-fail3
      else:
          ham3=pi-1.5707-fail3
      shank3=180*shank3/pi
      ham3=180*ham3/pi

      #Leg4
      x4=-x4
      shank4=pi-acos((x4*x4+y4*y4-l1*l1-l2*l2)/(-2*l1*l2))
      fai4=acos((l1*l1+x4*x4+y4*y4-l2*l2)/(2*l1*sqrt(x4*x4+y4*y4)))
      if x4>0:
          ham4=abs(atan(y4/x4))-fai4
      elif x4<0:
          ham4=pi-abs(atan(y4/x4))-fai4
      else:
          ham4=pi-1.5707-fai4

      shank4=180*shank4/pi
      ham4=180*ham4/pi
      
      return ham1,ham2,ham3,ham4,shank1,shank2,shank3,shank4






