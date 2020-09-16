/*
 * =====================================================================================
 *
 *       Filename:  filter4s4.cxx
 *
 *    Description:  filter
 *
 *        Version:  1.0
 *        Created:  12/05/2018 05:09:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  zhwu (mn), zhwu@robosense.cn
 *        Company:  RoboSense
 *
 * =====================================================================================
 */

#include "filter4s4.h"
#include <cmath>

filter4s4::filter4s4(void)
{
  mNodeNum = 0;
  cnt_vld = 0;
  cnt_inv = 0;
  validTh = 200;
  edgeTh = 50;
  fTmp = 0;
  fPre = 0;
}


void filter4s4::setOutNode(int nodeNum)
{
  if (nodeNum > 0 && nodeNum < MAX_NODE)
  {
    mNodeNum = nodeNum;
  }
  else
  {
    mNodeNum = 3;
  }
}

double filter4s4::isTrue_detect(double fInput)
{
  double difv = 0;

  if (fInput  <= 200)
  {
    difv = 65535;
  }
  else if (fTmp > 200)
  {
    difv = std::abs(fInput - fTmp);
  }
  else
  {
    difv = 0;
  }

  fTmp = fInput;
  isTrue = (fInput > 200) && (difv <= 3000);

  return isTrue;
}

double filter4s4::edge_detect(double fInput)
{
  if (fInput > validTh)
  {
    if (fPre == 0)
    {
      flag_edge = 0;
      fPre = fInput;
    }
    else if (std::abs(fInput - fPre) > edgeTh)
    {
      flag_edge = 1;
      fPre = fInput;
    }
    else
    {
      flag_edge = 0;
      fPre = fInput;
    }
  }
  else
  {
    flag_edge = 0;
  }

  return flag_edge;
}

double filter4s4::run(double inFilterPt)
{
  isTrue = isTrue_detect(inFilterPt);
  flag_edge = edge_detect(inFilterPt);

  if (isTrue == 1)
  {
    if (flag_edge == 0)
    {
      cnt_inv = 0;
      cnt_vld ++;
      for (int j = 0; j < 4; j++)
      {
        s[j][3] = s[j][2];
        s[j][2] = s[j][1];
        s[j][1] = s[j][0];
        s[j][0] = (j == 0)? inFilterPt: 0.25*(s[j-1][0] + s[j-1][1] + s[j-1][2] + s[j-1][3]);
      }
    }
    else
    {
      cnt_inv = 0;
      cnt_vld = 0;
      for (int j = 0; j < 4; j++)
      {
        s[j][3] = s[j][3];
        s[j][2] = s[j][3];
        s[j][1] = s[j][1];
        s[j][0] = s[j][0];
      }
    }
  }
  else
  {
    for (int j = 0; j < 4; j++)
    {
      s[j][3] = s[j][3];
      s[j][2] = s[j][3];
      s[j][1] = s[j][1];
      s[j][0] = s[j][0];
    }
    cnt_inv ++;
    cnt_vld = 0;
  }
      
  if (isTrue == 0)
  {
    fOut = 0;
  }
  else if (mNodeNum < 1)
  {
    fOut = inFilterPt;
  }
  else
  {
    fOut = 0.25*(s[mNodeNum-1][0] + s[mNodeNum-1][1] + s[mNodeNum-1][2] + s[mNodeNum-1][3]);
  }

  if (flag_edge == 1)
  {
    mNodeNum = 0;
  }
  else if (cnt_inv == 3)
  {
    mNodeNum = 0;
  }
  else if (cnt_inv == 2)
  {
    if (mNodeNum >= 1)
    {
      mNodeNum = 1;
    }
  }
  else if (cnt_inv == 1)
  {
    if (mNodeNum >= 2)
    {
      mNodeNum = 2;
    }
  }
  else
  {
    if (cnt_vld == 4 || cnt_vld == 8 || cnt_vld == 12 || cnt_vld == 16)
    {
      if (mNodeNum >= 4)
      {
        mNodeNum = 4;
      }
      else
      {
        mNodeNum ++;
      }
    }
    else
    {
      mNodeNum = mNodeNum;
    }
  }

  return fOut;
}


