#ifndef __SCREENGRABBER_H__
#define __SCREENGRABBER_H__

#include <FreeImage.h>
#include <windows.h>
#include <GL/glew.h>
#include "GL/gl.h"

class ScreenGrabber
{

   public:
      ScreenGrabber();
      void Grab(const char* filename = 0);
      void NextClip()   {ClipNumber++; FrameNumber=0;}

   protected:

      GLubyte* LpBuffer;
      int Width, Height;
      int Bpp; //bytes per pixel

      int FrameNumber;
      int ClipNumber;

};

class TiledGrabber
{
   public:
      TiledGrabber(int h_tiles, int v_tiles, int tile_w, int tile_h);
      void GrabTile(int i, int j);
      void Save(const char* filename = 0);

   protected:
      int mHTiles;
      int mVTiles;
      int mTileWidth;
      int mTileHeight;      
     
      FIBITMAP* mImage; 
};

#endif