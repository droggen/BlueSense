#include "cpu.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "a3d.h"

/*
	A3D buffer organised in lines terminated by CR and null.
*/
unsigned char _a3d_buffer[AFBRES_X*AFBRES_Y];

const float _height = 0.2;

float vcoord[]={
        // Bottom face
        -0.5, -0.5, -_height/2,     // 1
        -0.5, 0.5, -_height/2,      // 2
        0.5,0.5,-_height/2,       // 3
        0.5,-0.5,-_height/2,      // 4
        
        // Top face
        -0.5,-0.5,+_height/2,     // 5
        -0.5,0.5,+_height/2,      // 6
        0.5,0.5,+_height/2,       // 7
        0.5,-0.5,+_height/2,      // 8
        
        // Bluetooth
        -0.4,+0.5,+_height/2,    // 9
        -0.4,+0.1,+_height/2,    // 10
        +0.4,+0.1,+_height/2,    // 11
        +0.4,+0.5,+_height/2,    // 12
        
        // USB
        -0.1,-0.4,+_height/2,    // 13
        -0.1,-0.5,+_height/2,    // 14
        +0.1,-0.5,+_height/2,    // 15
        +0.1,-0.4,+_height/2,    // 16
    };


unsigned char vconnect[] = {
    // Bottom face
    1, 2,
    2, 3,
    3, 4,
    4, 1,
    // Top face
    5, 6,
    6, 7,
    7, 8,
    8, 5,
    // Link top and bottom faces
    1, 5,
    2, 6,
    3, 7,
    4, 8,
    
    // Bluetooth
    9, 10,
    10, 11,
    11, 12,
    12, 9,
    // USB
    13, 14,
    14, 15,
    15, 16,
    16, 13
    };
/*
vcol=[
    % Bottom
    'r';
    'r';
    'r';
    'r';
    % Top
    'b';
    'b';
    'b';
    'b';    
    % Links
    'k';
    'k';
    'k';
    'k';
    % Bluetooth
    'k';
    'k';
    'k';
    'k';
     % USB
    'k';
    'k';
    'k';
    'k';
    ];
*/



void a3d_clear(void)
{
	memset(_a3d_buffer,32,AFBRES_X*AFBRES_Y);
	/*for(unsigned y=0;y<AFBRES_Y;y++)
	{
		_a3d_buffer[y*
	}*/
}


void a3d_display(FILE *f)
{
	for(unsigned y=0;y<AFBRES_Y;y++)
	{
		for(unsigned x=0;x<AFBRES_X;x++)
		{
			fputc(*A3DPTR(x,y),f);
		}
		fputc('\n',f);
	}
	
}

void a3d_pixel(short x,short y,unsigned char p)
{
	if(x<0 || y<0 || x>=AFBRES_X || y>=AFBRES_Y)
		return;
		
	*A3DPTR(x,y)=p;
}
/*void a3d_line(short x0,short y0,short x1,short y1,unsigned char p)
{
	short dx = x1 - x0;
	short dy = y1 - y0;
	short D = 2*dy - dx;
	short y = y0;

	for(short x=x0;x<=x1;x++)
	{
		a3d_pixel(x,y,p);
		if(D > 0)
		{
			y = y + 1;
			D = D - 2*dx;
		}
		D = D + 2*dy;
	}
}*/

void a3d_line(short x1,short y1,short x2,short y2,unsigned char p)
{
	unsigned char steep = abs(y2 - y1) > abs(x2 - x1);
    signed short inc = -1;

	if (steep)
	{
		A3DSWAP(x1,y1);
		A3DSWAP(x2,y2);
	}

	if (x1 > x2)
	{
		A3DSWAP(x1,x2);
		A3DSWAP(y1,y2);
	}

	if (y1 < y2)
	{
		inc = 1;
	}

	short dx = abs(x2 - x1);
    short dy = abs(y2 - y1);
	short y = y1, x = x1, e = 0;

	for (x=x1; x <= x2; x++)
	{
		if (steep) 
		{
			a3d_pixel(y, x, p);
		} 
		else 
		{
			a3d_pixel(x, y, p);
		}

		if ((e + dy) << 1 < dx) 
		{
			e = e + dy;
		} 
		else 
		{
			y += inc;
			e = e + dy - dx;
		}
	}
}

void a3d_mat33multvect3(float *m,float *v,float *r)
{
	for(unsigned char j=0;j<3;j++)
	{
		r[j]=0;
		for(unsigned char i=0;i<3;i++)
		{
			r[j]=r[j]+m[i]*v[i];
		}
		m+=3;
	}
}
void a3d_proj3to2(float x1,float y1,float z1,float &px,float &py)
{
	float m[9]={-0.7071,    0.7071,         0,
		-0.5000,   -0.5000,    0.7071,
		0.5000,    0.5000,    0.7071};

	float v[3]={x1,y1,z1};
	float r[3];
	a3d_mat33multvect3(m,v,r);
	px=r[0];
	py=r[1];
	
	px*=10;
	py*=10;
	py=-py;
	px+=AFBRES_X/2;
	py+=AFBRES_Y/2;
}

void a3d_line3(short x1,short y1,short z1,short x2,short y2,short z2,unsigned char p)
{
	float px1,py1,px2,py2;
	a3d_proj3to2(x1,y1,z1,px1,py1);
	a3d_proj3to2(x2,y2,z2,px2,py2);
	
	/*px1+=AFBRES_X/2;
	py1+=AFBRES_Y/2;
	px2+=AFBRES_X/2;
	py2+=AFBRES_Y/2;
	*/
	
	
	a3d_line(px1,py1,px2,py2,p);
}
void a3d_line3f(float x1,float y1,float z1,float x2,float y2,float z2,unsigned char p)
{
	float px1,py1,px2,py2;
	a3d_proj3to2(x1,y1,z1,px1,py1);
	a3d_proj3to2(x2,y2,z2,px2,py2);
	
	/*px1+=AFBRES_X/2;
	py1+=AFBRES_Y/2;
	px2+=AFBRES_X/2;
	py2+=AFBRES_Y/2;
	*/
	
	
	a3d_line(px1,py1,px2,py2,p);
}



void a3d_quatmult(float *q1,float *q2,float *r)
{
    r[0] = (q1[0]*q2[0] -q1[1]*q2[1] -q1[2]*q2[2] -q1[3]*q2[3]);
    r[1] = (q1[0]*q2[1] +q1[1]*q2[0] +q1[2]*q2[3] -q1[3]*q2[2]);
    r[2] = (q1[0]*q2[2] -q1[1]*q2[3] +q1[2]*q2[0] +q1[3]*q2[1]);
    r[3] = (q1[0]*q2[3] +q1[1]*q2[2] -q1[2]*q2[1] +q1[3]*q2[0]);
}


void a3d_quatpointmult(float *q1,float *q2,float *result)
{
    result[0] = (q1[0]*q2[1] +q1[1]*q2[0] +q1[2]*q2[3] -q1[3]*q2[2]);
    result[1] = (q1[0]*q2[2] -q1[1]*q2[3] +q1[2]*q2[0] +q1[3]*q2[1]);
    result[2] = (q1[0]*q2[3] +q1[1]*q2[2] -q1[2]*q2[1] +q1[3]*q2[0]);
}

void a3d_quatrot(float *q,float *v,float *result)
{
    float qconj[4],qv[4],r[4];

    qv[0] = 0;
    qv[1] = v[0];
    qv[2] = v[1];
    qv[3] = v[2];
    qconj[0] = q[0];
    qconj[1] = -q[1];
    qconj[2] = -q[2];
    qconj[3] = -q[3];
    
	a3d_quatmult(q,qv,r);
	a3d_quatpointmult(r,qconj,result);
}

void a3d_drawgeom(float *q)
{
	for(unsigned char i=0;i<sizeof(vconnect)/sizeof(char);i+=2)
	{
		// Get the indices
		unsigned char i1 = vconnect[i]-1;
		unsigned char i2 = vconnect[i+1]-1;
		float p1[3],p2[3];
		float p1r[3],p2r[3];
		
		i1 = i1*3;
		i2 = i2*3;
		//printf("i1 i2 %d %d\n",i1,i2);
		
		float m=4;
	
		p1[0] = vcoord[i1]*m;
		p1[1] = vcoord[i1+1]*m;
		p1[2] = vcoord[i1+2]*m;
		p2[0] = vcoord[i2]*m;
		p2[1] = vcoord[i2+1]*m;
		p2[2] = vcoord[i2+2]*m;
		
		a3d_quatrot(q,p1,p1r);
		a3d_quatrot(q,p2,p2r);
		
		//printf("%f %f %f - %f %f %f\n",x1,y1,z1,x2,y2,z2);
		
		//a3d_line3(x1,y1,z1,x2,y2,z2,'.');
		a3d_line3f(p1r[0],p1r[1],p1r[2],p2r[0],p2r[1],p2r[2],'*');
		
	}	
}



/*

		//	------------------------------------
			if(ds++>75)
			{
				a3d_clear();
				
				a3d_line(0,0,AFBRES_X-1,0,'*');
				a3d_line(0,AFBRES_Y-1,AFBRES_X-1,AFBRES_Y-1,'*');
				a3d_line(0,0,0,AFBRES_Y-1,'*');
				a3d_line(AFBRES_X-1,0,AFBRES_X-1,AFBRES_Y-1,'*');
				
				a3d_line3(0,0,0,0,0,1,'Z');
				a3d_line3(0,0,0,0,1,0,'Y');
				a3d_line3(0,0,0,1,0,0,'X');
				
				float q[4];
				q[0]=q0;
				q[1]=q1;
				q[2]=q2;
				q[3]=q3;
				
				a3d_drawgeom(q);
				a3d_display(file_pri);
			
				ds=0;
			}
			
			//	------------------------------------
	*/
