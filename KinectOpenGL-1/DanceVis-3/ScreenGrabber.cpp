#include <windows.h>
#include "ScreenGrabber.h"
#include <stdio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glext.h>

ScreenGrabber::ScreenGrabber()
{
   FrameNumber = 0;
   ClipNumber = 0;
   Width = glutGet(GLUT_WINDOW_WIDTH);
   Height = glutGet(GLUT_WINDOW_HEIGHT);
   const int bw = glutGet(GLUT_WINDOW_BORDER_WIDTH); 
   const int hh = glutGet(GLUT_WINDOW_HEADER_HEIGHT); 

   Bpp = 3;

   LpBuffer = new GLubyte[Width*Height*Bpp];
   FreeImage_Initialise(1);

}

void ScreenGrabber::Grab(const char* filename)
{
   glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
   glReadBuffer(GL_BACK); //or back?
   glPixelStorei(GL_PACK_ALIGNMENT, 1);
   glReadPixels(0, 0, Width, Height, GL_BGR, GL_UNSIGNED_BYTE, LpBuffer);
   
   FIBITMAP* image = FreeImage_ConvertFromRawBits(LpBuffer, Width, Height, 3 * Width, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);

   if(filename)
   {
      FreeImage_Save(FIF_PNG, image, filename, 0);
   }
   else
   {
      char filenumber[256];
      sprintf(filenumber, "grabs/%d-%d.png", ClipNumber, FrameNumber);
      FreeImage_Save(FIF_PNG, image, filenumber, 0);
   }
   FreeImage_Unload(image);
   FrameNumber++;
}

TiledGrabber::TiledGrabber(int h_tiles, int v_tiles, int tile_w, int tile_h):
mHTiles(h_tiles), mVTiles(v_tiles), mTileWidth(tile_w), mTileHeight(tile_h)
{
   const int Width = mHTiles*mTileWidth;
   const int Height = mVTiles*mTileHeight;
   mImage = FreeImage_Allocate(Width, Height, 24, 0x0000FF, 0xFF0000, 0x00FF00);  
}

void TiledGrabber::GrabTile(int i, int j)
{
   const int Bpp = 3;
   GLubyte* LpBuffer = new GLubyte[mTileWidth*mTileHeight*Bpp];
 

   glReadBuffer(GL_FRONT);
   glPixelStorei(GL_PACK_ALIGNMENT, 1);
   glReadPixels(0, 0, mTileWidth, mTileHeight, GL_BGR, GL_UNSIGNED_BYTE, LpBuffer);
   
   FIBITMAP* tile = FreeImage_ConvertFromRawBits(LpBuffer, mTileWidth, mTileHeight, 3 * mTileWidth, 24, 0x0000FF, 0xFF0000, 0x00FF00, true);
   FreeImage_Paste(mImage, tile, i*mTileWidth, j*mTileHeight, 256);
   delete [] LpBuffer;  
}

void TiledGrabber::Save(const char* filename)
{
   FreeImage_Save(FIF_PNG, mImage, filename, 0);
}