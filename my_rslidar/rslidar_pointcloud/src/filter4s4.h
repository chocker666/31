/*
 * =====================================================================================
 *
 *       Filename:  filter4s4.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  12/05/2018 05:38:45 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  zhwu (mn), zhwu@robosense.cn
 *        Company:  RoboSense
 *
 * =====================================================================================
 */

#ifndef _FILTER4S4_H_
#define _FILTER4S4_H_

#define MAX_NODE    4

class filter4s4{
  public:
    double run(double inFilterPt);
    void setOutNode(int nodeNum);
    filter4s4();

    double validTh;
    double edgeTh;

  private:
    double s[MAX_NODE][4];

    int mNodeNum;
    int cnt_vld;
    int cnt_inv;
    int isTrue;
    int flag_edge;

    double fTmp;
    double fPre;
    double fOut;

    double isTrue_detect(double fInput);
    double edge_detect(double fInput);
};

#endif

