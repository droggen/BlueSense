#ifndef __A3D_H
#define __A3D_H

#define AFBRES_X	64
#define AFBRES_Y	48

#define A3DPTR(x,y) (_a3d_buffer+y*AFBRES_X+x)
#define A3DSWAP(x,y) x=x^y;y=y^x;x=x^y;


void a3d_clear(void);
void a3d_display(FILE *f);
void a3d_pixel(short x,short y,unsigned char p);
void a3d_line(short x0,short y0,short x1,short y1,unsigned char p);
//void a3d_line2(short x1,short y1,short x2,short y2,unsigned char p);
void a3d_mat33multvect3(float *m,float *v,float *r);
void a3d_line3(short x1,short y1,short z1,short x2,short y2,short z2,unsigned char p);
void a3d_proj3to2(float x1,float y1,float z1,float &px,float &py);

void a3d_quatmult(float *q1,float *q2,float *r);
void a3d_quatpointmult(float *q1,float *q2,float *result);
void a3d_quatrot(float *q,float *v,float *result);
void a3d_drawgeom(float *q);

#endif
