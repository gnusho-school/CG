# -*- coding: utf-8 -*-
import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import time

v_input=[]
vertex_array=None
vertex_normal_tmp=[]
itmp=[]
ntmp=[]
arr=None
use_for_array=[]

v_input_=[]
vertex_array_=None
vertex_normal_tmp_=[]
itmp_=[]
ntmp_=[]
arr_=None
use_for_array_=[]

#yes S var -> I will use gldrawelements
Varr=None
Narr=None
Iarr=None
ob=True
Z_val=GL_FILL

matrix=np.array([[1.,0.,0.,0.],
                [0.,1.,0.,0.],
                [0.,0.,1.,0.],
                [0.,0.,0.,1.]])

place=np.array([0.,1.,0.,1.])
#s1_place=np.array([5.,1.,7.5,1.])
s1_place=np.array([3.75,1.,-2.5,1.])
s3_place=np.array([7.5,1.,-5.,1.])
s2_place=np.array([-7.5,1.,5.,1.])
direct=np.array([1.,0.,0.,0.])

view=0
dist=1.5

lin_cnt1=0
moving1=False
after1=None
direct1=None

lin_cnt2=0
moving2=False
after2=None
direct2=None

lin_cnt3=0
moving3=False
after3=None
direct3=None

out1=False
out2=False
out3=False

p0 = np.array([0.,0.,0.])*7
p1 = np.array([2.,3.,1.])*7
p2 = np.array([2.,-3.,1.])*7
p3 = np.array([1.,0.,2.])*7
gEditingPoint = ''

fish=0
signal=True
M=False
def bezier_cal(t):
    global p0,p1,p2,p3

    tmp0=((1-t)**3)*p0 #k=0, n=3
    tmp1=(3*t*(1-t)**2)*p1 #k=1, n=3
    tmp2=(3*(t**2)*(1-t))*p2 #k=2, n=3
    tmp3=(t**3)*p3
    p=tmp1+tmp2+tmp3+tmp0
    return p

def drawCube():
    glBegin(GL_TRIANGLES)
    glNormal3f(0,0,1) # v0, v2, v1, v0, v3, v2normal
    glVertex3f( -1 , 1 , 1 ) # v0 position
    glVertex3f( 1 , -1 , 1 ) # v2 position
    glVertex3f( 1 , 1 , 1 ) # v1 position
    glVertex3f( -1 , 1 , 1 ) # v0 position
    glVertex3f( -1 , -1 , 1 ) # v3 position
    glVertex3f( 1 , -1 , 1 ) # v2 position
    glNormal3f(0,0,-1)
    glVertex3f( -1 , 1 , -1 ) # v4
    glVertex3f( 1 , 1 , -1 ) # v5
    glVertex3f( 1 , -1 , -1 ) # v6
    glVertex3f( -1 , 1 , -1 ) # v4
    glVertex3f( 1 , -1 , -1 ) # v6
    glVertex3f( -1 , -1 , -1 ) # v7
    glNormal3f(0,1,0)
    glVertex3f( -1 , 1 , 1 ) # v0
    glVertex3f( 1 , 1 , 1 ) # v1
    glVertex3f( 1 , 1 , -1 ) # v5
    glVertex3f( -1 , 1 , 1 ) # v0
    glVertex3f( 1 , 1 , -1 ) # v5
    glVertex3f( -1 , 1 , -1 ) # v4
    glNormal3f(0,-1,0)
    glVertex3f( -1 , -1 , 1 ) # v3
    glVertex3f( 1 , -1 , -1 ) # v6
    glVertex3f( 1 , -1 , 1 ) # v2
    glVertex3f( -1 , -1 , 1 ) # v3
    glVertex3f( -1 , -1 , -1 ) # v7
    glVertex3f( 1 , -1 , -1 ) # v6
    glNormal3f(1,0,0)
    glVertex3f( 1 , 1 , 1 ) # v1
    glVertex3f( 1 , -1 , 1 ) # v2
    glVertex3f( 1 , -1 , -1 ) # v6
    glVertex3f( 1 , 1 , 1 ) # v1
    glVertex3f( 1 , -1 , -1 ) # v6
    glVertex3f( 1 , 1 , -1 ) # v5
    glNormal3f(-1,0,0)
    glVertex3f( -1 , 1 , 1 ) # v0
    glVertex3f( -1 , -1 , -1 ) # v7
    glVertex3f( -1 , -1 , 1 ) # v3
    glVertex3f( -1 , 1 , 1 ) # v0
    glVertex3f( -1 , 1 , -1 ) # v4
    glVertex3f( -1 , -1 , -1 ) # v7
    glEnd()

def draw_main():
    global matrix
    t=glfw.get_time()
    a=t%45
    drawFrame()
    glColor3ub(255, 255, 255)

    glPushMatrix()

    t = glfw.get_time()

    n = t
    if t>1:
        n=t%1

    if n>0.5:
        n=1-n

    a=t*(180/np.pi)
    if a>180: 
        a=a%180

    if a<90:
        a=180-a
 
    #hand main
    glPushMatrix()
    glMultMatrixf(matrix.T)
    
    glPushMatrix()
    glScalef(.1, .9, .7)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #first finger 1
    glPushMatrix()
    glTranslatef(.15, 1, .6) 
    glRotatef(-a/2, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(-.15, 0., 0.) 
    glScalef(.15, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #first finger 2
    glPushMatrix() 
    glTranslatef(-.3, 0., 0.)
    glRotatef((270-a), 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.15, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()

    #second finger 1
    glPushMatrix()
    glTranslatef(.15, 1, .3) 
    glRotatef(-a/2, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(-.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #second finger 2
    glPushMatrix() 
    glTranslatef(-.4, 0., 0.)
    glRotatef((270-a), 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #second finger 3
    glPushMatrix() 
    glTranslatef(.4, 0., 0.)
    glRotatef(90-a, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()
    glPopMatrix()

    #third finger 1
    glPushMatrix()
    glTranslatef(.15, 1, 0.) 
    glRotatef(-a/2, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(-.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #third finger 2
    glPushMatrix() 
    glTranslatef(-.4, 0., 0.)
    glRotatef((270-a), 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #third finger 3
    glPushMatrix() 
    glTranslatef(.4, 0., 0.)
    glRotatef(90-a, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()
    glPopMatrix()
    
    #fourth finger 1
    glPushMatrix()
    glTranslatef(.15, 1, -.3) 
    glRotatef(-a/2, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(-.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #fourth finger 2
    glPushMatrix() 
    glTranslatef(-.4, 0., 0.)
    glRotatef((270-a), 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #fourth finger 3
    glPushMatrix() 
    glTranslatef(.4, 0., 0.)
    glRotatef(90-a, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.2, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()
    glPopMatrix()

    #fifth finger 1
    glPushMatrix()
    glTranslatef(.15, 1, -.6) 
    glRotatef(-a/2, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(-.2, 0., 0.) 
    glScalef(.15, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #fifth finger 2
    glPushMatrix() 
    glTranslatef(-.3, 0., 0.)
    glRotatef((270-a), 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.15, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    #fifth finger 3
    glPushMatrix() 
    glTranslatef(.3, 0., 0.)
    glRotatef(90-a, 0, 0, 1) 
 
    glPushMatrix()
    glTranslatef(.2, 0., 0.) 
    glScalef(.15, .1, .1)
    glColor3ub(255, 255, 255)
    drawCube()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()

def square():

    global v_input, vertex_array, vertex_normal_tmp, arr, itmp, ntmp, ob
    global Varr, Narr, Iarr
    v_input=[]
    vertex_array=None
    vertex_normal_tmp=[]
    arr=None
    itmp=[]
    ntmp=[]
    
    Varr=None
    Narr=None
    Iarr=None

    ob=True
    f3=0
    f4=0
    f5=0
    #reset the data
    f=open("sphere-tri.obj",'r')
    while True:
        line=f.readline()
        if not line:
            break
        sp=line.split()

        if len(sp)<=0:
            continue
        #print(sp[0])
        
        if sp[0]=="v": 
            vm=[float(sp[1]),float(sp[2]),float(sp[3])]
            v_input.append(vm)

        elif sp[0]=="vn":
            vnm=[float(sp[1]),float(sp[2]),float(sp[3])]
            vertex_normal_tmp.append(vnm)

        elif sp[0]=="f":
            if len(sp)-1==3:
                f3+=1
            elif len(sp)-1==4:
                f4+=1
            elif len(sp)-1>4:
                f5+=1

            for i in range(len(sp)-3):
                i1=sp[1].split('/')
                i2=sp[i+2].split('/')
                i3=sp[i+3].split('/')
                im=[int(i1[0])-1,int(i2[0])-1,int(i3[0])-1]
                nm=[int(i1[2])-1,int(i2[2])-1,int(i3[2])-1]
                itmp.append(im)
                ntmp.append(nm)
            #위의 배열이 0번지부터 들어갔으니까 이것도 1씩 빼줘야함
            #안 그러면 이상한 그림 나옴
            
    f.close()
    
    #print(len(itmp))
    use_for_array=[]
    for i in range(len(itmp)):
        for j in range(3):
            use_for_array.append(vertex_normal_tmp[ntmp[i][j]])
            use_for_array.append(v_input[itmp[i][j]])
    #그냥 저장하면 float64여서 에러가 났다.
    arr=np.array(use_for_array, 'float32')

def draw_square():
    global vertex_array,arr
    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_NORMAL_ARRAY)
    glNormalPointer(GL_FLOAT, 6*arr.itemsize, arr)
    glVertexPointer(3, GL_FLOAT, 6*arr.itemsize,ctypes.c_void_p(arr.ctypes.data + 3*arr.itemsize))
    glDrawArrays(GL_TRIANGLES, 0, int(arr.size/6))

def final_hand():
    global v_input_, vertex_array_, vertex_normal_tmp_, arr_, itmp_, ntmp_, ob
    v_input_=[]
    vertex_array_=None
    vertex_normal_tmp_=[]
    arr_=None
    itmp_=[]
    ntmp_=[]
    
    Varr=None
    Narr=None
    Iarr=None

    ob=True
    f3=0
    f4=0
    f5=0
    #reset the data
    f=open("Hand.obj",'r')
    while True:
        line=f.readline()
        if not line:
            break
        sp=line.split()

        if len(sp)<=0:
            continue
        #print(sp[0])
        
        if sp[0]=="v": 
            vm=[float(sp[1]),float(sp[2]),float(sp[3])]
            v_input_.append(vm)

        elif sp[0]=="vn":
            vnm=[float(sp[1]),float(sp[2]),float(sp[3])]
            vertex_normal_tmp_.append(vnm)

        elif sp[0]=="f":
            if len(sp)-1==3:
                f3+=1
            elif len(sp)-1==4:
                f4+=1
            elif len(sp)-1>4:
                f5+=1

            for i in range(len(sp)-3):
                i1=sp[1].split('/')
                i2=sp[i+2].split('/')
                i3=sp[i+3].split('/')
                im=[int(i1[0])-1,int(i2[0])-1,int(i3[0])-1]
                nm=[int(i1[2])-1,int(i2[2])-1,int(i3[2])-1]
                itmp_.append(im)
                ntmp_.append(nm)
            #위의 배열이 0번지부터 들어갔으니까 이것도 1씩 빼줘야함
            #안 그러면 이상한 그림 나옴
            
    f.close()
    
    #print(len(itmp_))
    use_for_array=[]
    for i in range(len(itmp_)):
        for j in range(3):
            use_for_array.append(vertex_normal_tmp_[ntmp_[i][j]])
            use_for_array.append(v_input_[itmp_[i][j]])
    #그냥 저장하면 float64여서 에러가 났다.
    arr_=np.array(use_for_array, 'float32')

def draw_hand():
    global vertex_array_,arr_
    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_NORMAL_ARRAY)
    glNormalPointer(GL_FLOAT, 6*arr_.itemsize, arr_)
    glVertexPointer(3, GL_FLOAT, 6*arr_.itemsize,ctypes.c_void_p(arr_.ctypes.data + 3*arr_.itemsize))
    glDrawArrays(GL_TRIANGLES, 0, int(arr_.size/6))

def render():
    global place, view, vertex_array,  ob, Z_val, arr, s1_place, s2_place, s3_place, direct, dist
    global lin_cnt1, moving1, after1, direct1, out1
    global lin_cnt2, moving2, after2, direct2, out2
    global lin_cnt3, moving3, after3, direct3, out3
    global fish, signal, M
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glPolygonMode( GL_FRONT_AND_BACK, Z_val )
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1, 1,1000)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_NORMALIZE)
    glLoadIdentity()

    #######################################################3
    if s1_place[0]>35 or s1_place[0]<-35:
        out1=True
    elif s1_place[2]>35 or s1_place[2]<-35:
        out1=True
    else: out1=False

    if s2_place[0]>35 or s2_place[0]<-35:
        out2=True
    elif s2_place[2]>35 or s2_place[2]<-35:
        out2=True
    else: out2=False

    if s3_place[0]>35 or s3_place[0]<-35:
        out3=True
    elif s3_place[2]>35 or s3_place[2]<-35:
        out3=True
    else: out3=False

    #ball and hand ##################################################3
    #red ball
    len1=(s1_place[0]-place[0])*(s1_place[0]-place[0])
    len2=(s1_place[1]-place[1])*(s1_place[1]-place[1])
    len3=(s1_place[2]-place[2])*(s1_place[2]-place[2])

    len_1=np.sqrt(len1+len2+len3)
    
    if len_1<dist:
        tmp=np.dot(direct,s1_place-place)
        hh=s1_place-place
        hh[1]=0.
        hh[3]=0.
        cos=tmp/len_1
        if cos<1 and cos>0:
            after1=s1_place+hh*cos*2.5
            normal=np.sqrt(hh[0]*hh[0]+hh[1]*hh[1]+hh[2]*hh[2])
            direct1=hh
            moving1=True

    #green ball
    len1=(s2_place[0]-place[0])*(s2_place[0]-place[0])
    len2=(s2_place[1]-place[1])*(s2_place[1]-place[1])
    len3=(s2_place[2]-place[2])*(s2_place[2]-place[2])

    len_2=np.sqrt(len1+len2+len3)
    
    if len_2<dist:
        tmp=np.dot(direct,s2_place-place)
        hh=s2_place-place
        hh[1]=0.
        hh[3]=0.
        cos=tmp/len_2
        if cos<1 and cos>0:
            after2=s2_place+hh*cos*2.5
            normal=np.sqrt(hh[0]*hh[0]+hh[1]*hh[1]+hh[2]*hh[2])
            direct2=hh
            moving2=True

    #blue ball
    len1=(s3_place[0]-place[0])*(s3_place[0]-place[0])
    len2=(s3_place[1]-place[1])*(s3_place[1]-place[1])
    len3=(s3_place[2]-place[2])*(s3_place[2]-place[2])

    len_3=np.sqrt(len1+len2+len3)
    
    if len_3<dist:
        tmp=np.dot(direct,s3_place-place)
        hh=s3_place-place
        hh[1]=0.
        hh[3]=0.
        cos=tmp/len_3
        if cos<1 and cos>0:
            after3=s3_place+hh*cos*2.5
            normal=np.sqrt(hh[0]*hh[0]+hh[1]*hh[1]+hh[2]*hh[2])
            direct3=hh
            moving3=True
    ################################################################
    #Red ball run and collide other ball
    if moving1==True:
        dx=np.linspace(0.,after1[0]-s1_place[0],100)
        dz=np.linspace(0.,after1[2]-s1_place[2],100)
        tmp1=s1_place
        tmp1[0]+=dx[lin_cnt1]
        tmp1[2]+=dz[lin_cnt1]

        len_12=np.sqrt(np.dot(tmp1-s2_place,tmp1-s2_place))
        len_13=np.sqrt(np.dot(tmp1-s3_place,tmp1-s3_place))

        if len_12<dist+1 and moving2==False:
            direct2=s2_place-tmp1
            sum2=np.sqrt(np.dot(direct2,direct2))
            direct2/=sum2
            sum1=np.sqrt(np.dot(direct1,direct1))
            direct1/=sum1
            aa=np.array([direct1[0],direct1[1],direct1[2]])
            bb=np.array([direct2[0],direct2[1],direct2[2]])
            cc=np.cross(aa,bb)
            theta=np.arccos(np.dot(direct1,direct2))
            if cc[1]<=0:
                th=np.pi/2-theta
            else: th=-(np.pi/2-theta)
            s1_place=tmp1
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])
            ttmp=after1-s1_place
            after1=s1_place+1.3*T@ttmp*(100-lin_cnt1)/200
            after2=s2_place+2*sum2*direct2*(100-lin_cnt1)/100
            moving2=True
            #s1_place=tmp1
        
        if len_13<dist+1 and moving3==False:
            direct3=s3_place-tmp1
            sum2=np.sqrt(np.dot(direct3,direct3))
            direct3/=sum2
            sum1=np.sqrt(np.dot(direct1,direct1))
            direct1/=sum1
            aa=np.array([direct1[0],direct1[1],direct1[2]])
            bb=np.array([direct3[0],direct3[1],direct3[2]])
            cc=np.cross(aa,bb)
            theta=np.arccos(np.dot(direct1,direct3))
            if cc[1]<=0:
                th=np.pi/2-theta
            else: th=-(np.pi/2-theta)
            s1_place=tmp1
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])
            ttmp=after1-s1_place
            after1=s1_place+1.3*T@ttmp*(100-lin_cnt1)/200
            after3=s3_place+2*sum2*direct3*(100-lin_cnt1)/100
            moving3=True

    #Green ball run and collide other ball
    elif moving2==True:
        dx=np.linspace(0.,after2[0]-s2_place[0],100)
        dz=np.linspace(0.,after2[2]-s2_place[2],100)
        tmp1=s2_place
        tmp1[0]+=dx[lin_cnt2]
        tmp1[2]+=dz[lin_cnt2]

        len_21=np.sqrt(np.dot(tmp1-s1_place,tmp1-s1_place))
        len_23=np.sqrt(np.dot(tmp1-s3_place,tmp1-s3_place))

        if len_21<dist+1 and moving1==False:
            direct1=s1_place-tmp1
            sum2=np.sqrt(np.dot(direct1,direct1))
            direct1/=sum2
            sum1=np.sqrt(np.dot(direct2,direct2))
            direct2/=sum1
            aa=np.array([direct2[0],direct2[1],direct2[2]])
            bb=np.array([direct1[0],direct1[1],direct1[2]])
            cc=np.cross(aa,bb)
            theta=np.arccos(np.dot(direct2,direct1))
            print(theta)
            if cc[1]<=0:
                th=np.pi/2-theta
            else: th=-(np.pi/2-theta)
            s2_place=tmp1
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])
            ttmp=after2-s2_place
            after2=s2_place+1.3*T@ttmp*(100-lin_cnt2)/200
            after1=s1_place+2*sum2*direct1*(100-lin_cnt2)/100
            moving1=True
            #s1_place=tmp1
        
        if len_23<dist+1 and moving3==False:
            direct3=s3_place-tmp1
            sum2=np.sqrt(np.dot(direct3,direct3))
            direct3/=sum2
            sum1=np.sqrt(np.dot(direct2,direct2))
            direct2/=sum1
            aa=np.array([direct2[0],direct2[1],direct2[2]])
            bb=np.array([direct3[0],direct3[1],direct3[2]])
            cc=np.cross(aa,bb)
            theta=np.arccos(np.dot(direct2,direct3))
            if cc[1]<=0:
                th=np.pi/2-theta
            else: th=-(np.pi/2-theta)
            s2_place=tmp1
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])
            ttmp=after2-s2_place
            after2=s2_place+1.3*T@ttmp*(100-lin_cnt2)/200
            after3=s3_place+2*sum2*direct3*(100-lin_cnt2)/100
            moving3=True



    #Blue ball run and collide other ball
    elif moving3==True:
        dx=np.linspace(0.,after3[0]-s3_place[0],100)
        dz=np.linspace(0.,after3[2]-s3_place[2],100)
        tmp1=s3_place
        tmp1[0]+=dx[lin_cnt3]
        tmp1[2]+=dz[lin_cnt3]

        len_32=np.sqrt(np.dot(tmp1-s2_place,tmp1-s2_place))
        len_31=np.sqrt(np.dot(tmp1-s1_place,tmp1-s1_place))

        if len_32<dist+1 and moving2==False:
            direct2=s2_place-tmp1
            sum2=np.sqrt(np.dot(direct2,direct2))
            direct2/=sum2
            sum1=np.sqrt(np.dot(direct3,direct3))
            direct3/=sum1
            aa=np.array([direct3[0],direct3[1],direct3[2]])
            bb=np.array([direct2[0],direct2[1],direct2[2]])
            cc=np.cross(aa,bb)
            theta=np.arccos(np.dot(direct3,direct2))
            if cc[1]<=0:
                th=np.pi/2-theta
            else: th=-(np.pi/2-theta)
            s3_place=tmp1
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])
            ttmp=after3-s3_place
            after3=s3_place+1.3*T@ttmp*(100-lin_cnt3)/200
            after2=s2_place+2*sum2*direct2*(100-lin_cnt3)/100
            moving2=True
            #s1_place=tmp1
        
        if len_31<dist+1 and moving1==False:
            direct1=s1_place-tmp1
            sum2=np.sqrt(np.dot(direct1,direct1))
            direct1/=sum2
            sum1=np.sqrt(np.dot(direct3,direct3))
            direct3/=sum1
            aa=np.array([direct3[0],direct3[1],direct3[2]])
            bb=np.array([direct1[0],direct1[1],direct1[2]])
            cc=np.cross(aa,bb)
            theta=np.arccos(np.dot(direct3,direct1))
            if cc[1]<=0:
                th=np.pi/2-theta
            else: th=-(np.pi/2-theta)
            s3_place=tmp1
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])
            ttmp=after3-s3_place
            after3=s3_place+1.3*T@ttmp*(100-lin_cnt3)/200
            after1=s1_place+2*sum2*direct1*(100-lin_cnt3)/100
            moving1=True

    ################################################################
    if view==0:
        a=25
        gluLookAt(a+place[0],a+place[1],a+place[2], place[0],place[1],place[2], 0,1,0)
    
    elif view==1:
        gluLookAt(place[0],place[1],place[2],place[0]+direct[0],place[1]+direct[1],place[2]+direct[2],0,1,0)

    k=np.linspace(-35,35,50)
    glColor3ub(255,255,255)
    glBegin(GL_LINES)
    for i in range(0, 50):
            glVertex3f(-35,0,k[i])
            glVertex3f(35,0,k[i])
            glVertex3f(k[i],0,-35)
            glVertex3f(k[i],0,35)
    glEnd()

    #print(M)


    #######

    ################################################################
    glEnable(GL_LIGHTING) # try to uncomment: no lighting
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_NORMALIZE)

    glPushMatrix()
    lightPos = (-100.,-100.,-100.,1.) # try to change 4th element to 0. or 1.
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos)
    glPopMatrix()

    lightColor = (1.,1.,1.,1.)
    ambientLightColor = (.1,.1,.1,1.)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor)
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor)
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLightColor)

    t=glfw.get_time() 
    
    glPushMatrix()
    lightPos = (np.cos(t)*25,25.,np.sin(t)*25,0.) # try to change 4th element to 0. or 1.
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos)
    glPopMatrix()

    lightColor = (1.,0.,1.,1.)
    ambientLightColor = (.1,0.,.1,1.)
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor)
    glLightfv(GL_LIGHT1, GL_SPECULAR, lightColor)
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLightColor)

    #########################################################################
    objectColor = (1.,1.,1.,1.)
    specularObjectColor = (1.,1.,1.,1.)
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, objectColor)
    glMaterialfv(GL_FRONT, GL_SHININESS, 10)
    glMaterialfv(GL_FRONT, GL_SPECULAR, specularObjectColor)
    
    draw_main()
    
    ##########################################################################
    objectColor = (1.,0.,0.,1.)
    specularObjectColor = (1.,0.,0.,1.)
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, objectColor)
    glMaterialfv(GL_FRONT, GL_SHININESS, 10)
    glMaterialfv(GL_FRONT, GL_SPECULAR, specularObjectColor)
    
    glPushMatrix()
    if moving1==False:
        glTranslatef(s1_place[0],s1_place[1],s1_place[2])
        #print(s1_place)
        draw_square()
    else:
        dx=np.linspace(0.,after1[0]-s1_place[0],100)
        dz=np.linspace(0.,after1[2]-s1_place[2],100)
        #print([s1_place[0]+dx[lin_cnt1],s1_place[1],s1_place[2]+dz[lin_cnt1]])
        glTranslatef(s1_place[0]+dx[lin_cnt1],s1_place[1],s1_place[2]+dz[lin_cnt1])
        glRotatef(t,1.,0.,1.)
        draw_square()
        lin_cnt1+=1
        if lin_cnt1==100: 
            s1_place=after1
            after1=None
            moving1=False
            lin_cnt1=0

    glPopMatrix()

    #############################################################
    objectColor = (0.,1.,0.,1.)
    specularObjectColor = (0.,1.,0.,1.)
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, objectColor)
    glMaterialfv(GL_FRONT, GL_SHININESS, 10)
    glMaterialfv(GL_FRONT, GL_SPECULAR, specularObjectColor)

    glPushMatrix()
    if moving2==False:
        glTranslatef(s2_place[0],s2_place[1],s2_place[2])
        #print(s2_place)
        draw_square()
    else:
        dx=np.linspace(0.,after2[0]-s2_place[0],100)
        dz=np.linspace(0.,after2[2]-s2_place[2],100)
        #print([s2_place[0]+dx[lin_cnt2],s2_place[1],s2_place[2]+dz[lin_cnt2]])
        glTranslatef(s2_place[0]+dx[lin_cnt2],s2_place[1],s2_place[2]+dz[lin_cnt2])
        glRotatef(t,1.,0.,1.)
        draw_square()
        lin_cnt2+=1
        if lin_cnt2==100: 
            s2_place=after2
            after2=None
            moving2=False
            lin_cnt2=0
    glPopMatrix()

    ####################################################
    objectColor = (0.,0.,1.,1.)
    specularObjectColor = (0.,0.,1.,1.)
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, objectColor)
    glMaterialfv(GL_FRONT, GL_SHININESS, 10)
    glMaterialfv(GL_FRONT, GL_SPECULAR, specularObjectColor)

    glPushMatrix()
    if moving3==False:
        glTranslatef(s3_place[0],s3_place[1],s3_place[2])
        #print(s3_place)
        draw_square()
    else:
        dx=np.linspace(0.,after3[0]-s3_place[0],100)
        dz=np.linspace(0.,after3[2]-s3_place[2],100)
        #print([s3_place[0]+dx[lin_cnt3],s3_place[1],s3_place[2]+dz[lin_cnt3]])
        glTranslatef(s3_place[0]+dx[lin_cnt3],s3_place[1],s3_place[2]+dz[lin_cnt3])
        glRotatef(t,1.,0.,1.)
        draw_square()
        lin_cnt3+=1
        if lin_cnt3==100: 
            s3_place=after3
            after3=None
            moving3=False
            lin_cnt3=0

    glPopMatrix()

    objectColor = (1.,1.,1.,1.)
    specularObjectColor = (1.,1.,1.,1.)
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, objectColor)
    glMaterialfv(GL_FRONT, GL_SHININESS, 10)
    glMaterialfv(GL_FRONT, GL_SPECULAR, specularObjectColor)

    if (out1==True and out2==True and out3==True) or M==True:

        t=np.linspace(0,1,100)
        p = bezier_cal(t[fish])
        glPushMatrix()
        glTranslatef(p[0],p[1],p[2])
        glTranslatef(2.,0.,-1.)
        glScalef(.4,.4,.4)
        glRotatef(90,0,0,-1)
        draw_hand()
        glPopMatrix()

        if signal==True:
            fish+=1
        else: fish-=1
        if fish==99:
            signal=False
        elif fish==0:
            signal=True

        print("Thank You! TA")

    glDisable(GL_LIGHTING)

    
         

def key_callback(window, key, scancode, action, mods):
    global Z_val, S, matrix, place, view, direct, dist, M, view
    if action==glfw.PRESS:
        if key==glfw.KEY_Z:
            if Z_val==GL_LINE:
                Z_val=GL_FILL
            elif Z_val==GL_FILL:
                Z_val=GL_LINE

        if key==glfw.KEY_SPACE:
            if S==1:
                S=0
            elif S==0:
                S=1

        if key==glfw.KEY_W:
            T=np.array([[1.,0.,0.,.5],
                        [0.,1.,0.,0.],
                        [0.,0.,1.,0.],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T

        if key==glfw.KEY_S:
            T=np.array([[1.,0.,0.,-.5],
                        [0.,1.,0.,0.],
                        [0.,0.,1.,0.],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T

        if key==glfw.KEY_D:
            T=np.array([[1.,0.,0.,0.],
                        [0.,1.,0.,0.],
                        [0.,0.,1.,.5],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T

        if key==glfw.KEY_A:
            T=np.array([[1.,0.,0.,0.],
                        [0.,1.,0.,0.],
                        [0.,0.,1.,-.5],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T

        if key==glfw.KEY_R:
            th=np.radians(5)
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T
            direct=T@direct

        if key==glfw.KEY_T:
            th=np.radians(-5)
            T=np.array([[np.cos(th),0.,np.sin(th),0.],
                        [0.,1.,0.,0.],
                        [-np.sin(th),0.,np.cos(th),0.],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T
            direct=T@direct

        if key==glfw.KEY_E:
            T=np.array([[-1.,0.,0.,0.],
                        [0.,1.,0.,0.],
                        [0.,0.,-1.,0.],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T
            direct=T@direct

        #scale
        if key==glfw.KEY_P:
            T=np.array([[1.1,0.,0.,0.],
                        [0.,1.1,0.,0.],
                        [0.,0.,1.1,0.],
                        [0.,0.,0.,1.]])
            matrix=matrix@T
            dist*=1.1

        if key==glfw.KEY_O:
            T=np.array([[.9,0.,0.,0.],
                        [0.,.9,0.,0.],
                        [0.,0.,.9,0.],
                        [0.,0.,0.,1.]])
            matrix=matrix@T
            dist*=0.9

        #shear
        if key==glfw.KEY_I:
            T=np.array([[1.,.1,0.,0.],
                        [0.,1.,0.,0.],
                        [0.,.1,1.,0.],
                        [0.,0.,0.,1.]])   
            matrix=matrix@T


        if key==glfw.KEY_V:
            if view==1:
                view=0
            elif view==0:
                view=1

        if key==glfw.KEY_M:
            if M==True:
                M=False
            elif M==False:
                M=True

        place=np.array([0.,1.,0.,1.])
        place=matrix@place

def drawFrame():
    glBegin(GL_LINES)
    glColor3ub(255, 0, 0)
    glVertex3fv(np.array([0.,0.,0.]))
    glVertex3fv(np.array([30,0.,0.]))
    glColor3ub(0, 255, 0)
    glVertex3fv(np.array([0.,0.,0.]))
    glVertex3fv(np.array([0.,30.,0.]))
    glColor3ub(0, 0, 255)
    glVertex3fv(np.array([0.,0.,0]))
    glVertex3fv(np.array([0.,0.,30.]))
    glEnd()

def main():
    if not glfw.init():
        return
    window = glfw.create_window(1000,1000,"2016024893", None,None)
    if not window:
        glfw.terminate()
        return

    glfw.set_key_callback(window, key_callback)

    glfw.make_context_current(window)

    glfw.swap_interval(1)

    square()
    final_hand()
    while not glfw.window_should_close(window):
        glfw.poll_events()

        render()

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()