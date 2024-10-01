#pragma once

#include <math.h>

struct combines_x_x_t
{
  float y1;//当前真实值
  float y2;//当前估计值
  //融合数据
  float combines_x_x(float x1,float x2,float k,float t)
  {
    y1 = y2 + x1 * t;
    y2 = k * x1 + (1.0 - k) * x2;
		return y2;
  }
};


struct bool_and_float
{
  bool x1;
  float x2,x3,x4;
  bool_and_float(float x2 = 0,float x3 = 0,float x4 = 0)
    :x1(true),x2(x2),x3(x3),x4(x4)
  {

  }
};

//线性/非线性函数enum
enum NonLinear_Fun_ENUM
{
  Linear = 0,
  Fast_TD,
  Fal,
  Fhan,
  Fign,
  Fsg,
};

//非线性函数
namespace NonLinear_Fun
{
  int Fign_Fun(float x)
  {
    if(x > 0)return 1;
    if(x < 0)return -1;
    return 0;
  }
  float Fal_Fun(float x,float a,float xigema)
  {
    if(fabsf(x) <= xigema)
    {
      return x / powf(xigema,1-a);
    }
    else
    {
      return Fign_Fun(x) * powf(fabsf(x),a);
    }
  }
  float Fsg_Fun(float x,float d)
  {
    return (Fign_Fun(x + d) - Fign_Fun(x - d)) * 0.5;
  }
  float fhan_d,fhan_a0,fhan_y,fhan_a1,fhan_a2,fhan_a;
  float Fhan_Fun(float x1,float x2,float r,float h)
  {
    fhan_d = r * h * h;
    fhan_a0 = h * x2;
    fhan_y = x1 + fhan_a0;
    fhan_a1 = sqrtf(fhan_d * (fhan_d + 8.0 * fabs(fhan_y)));
    fhan_a2 = fhan_a0 + Fign_Fun(fhan_y) * (fhan_a1 - fhan_d) * 0.5;
    fhan_a = (fhan_a0 + fhan_y) * Fsg_Fun(fhan_y,fhan_d) + fhan_a2 * (1.0 - Fsg_Fun(fhan_y,fhan_d));
    return -r * (fhan_a / fhan_d) * Fsg_Fun(fhan_a,fhan_d) - r * Fign_Fun(fhan_a) * (1.0 - Fsg_Fun(fhan_a,fhan_d));
  }
};

using namespace NonLinear_Fun;

//二阶微分跟踪器
struct TD_quadratic
{
	float u;
	float x1,x2,max_x2;
	float r,h,r2_1; 
  unsigned char Fun;
	TD_quadratic(float r = 1,float max_x2 = 0,float h = 0.001,unsigned char Fun = Linear)
	 :r(r),h(h),max_x2(max_x2),Fun(Fun)
	{
    
  }
	void clear()
	{
		this->x1 = this->x2 = 0;
	}
  //二阶线性跟踪微分器（TD）
  void td_quadratic(float u,bool is = true)
  {
    this->u = u;
    /*
    u = 输入值
    x = [x x']	
    r = 调节步长
    h = 间隔步长
    Fun 调用的函数
    */
    if(is == false)
    {
      this->x1 = this->u;
      this->x2 = 0;
      return;
    }
    switch (Fun)
    {
    case Linear:
      {
        this->x1 += this->x2 * this->h;
        this->x2 += (-2.0 * this->r * this->x2 - this->r * this->r * (this->x1 - this->u)) * this->h;
      }
      break;
    case Fast_TD:
      {
        this->r2_1 = 1.0 / (2.0 * this->r);//r2_1是2r的倒数
        float f = -this->r * Fign_Fun(this->x1 - this->u + (this->x2 * fabs(this->x2) * this->r2_1));
        this->x1 += this->x2 * this->h;
        this->x2 += f * this->h;
      }
      break;
    case Fhan:
      {
        float fh = Fhan_Fun(this->x1 - this->u,this->x2,this->r,this->h);
        this->x1 += this->x2 * this->h;
        this->x2 += fh * this->h;
      }
      break;
    default:
      break;
    }

    if(this->max_x2 != 0)
    {
      if(this->x2 > this->max_x2)this->x2 = this->max_x2;
      else if(this->x2 < -this->max_x2)this->x2 = -this->max_x2;
    }
  }
};

//二阶ladrc控制结构体
struct LADRC_quadratic
{
	/*
	假设位置输入
	z1 = 位置
	z2 = 位置一阶导
	z3 = 总扰动
	feedback = 反馈量
	u = 系统输出
	b = 系统参数(需调节)
	w0 = 控制器系统(kp,kd)
	wc = 观测器带宽
	h = 间隔步长
	e = 观测器z1与目标的误差
	max_u = 最大系统输出
	输入使用td的x1和x2
  is_separate 是否分离kp,kd和w0的关系....
  β bt1,bt2,bt3
  customize_rank1_z1_dt z1的倒数 用于一阶
  customize_rank2_z1_dt z1的倒数 用于二阶
  customize_rank2_z1_dt_k 系数
	*/
	TD_quadratic td;
	float z1,z2,z3;
	float feedback,u;
	float b,w0,wc;
	float e,e2;
	float max_u;
  float h;
  bool is_separate_wc,is_separate_w0;
  float kp,kd;
  float bt1,bt2,bt3,max_z2,max_z3;
  float customize_rank1_z1_dt;
  float feedback_z2;
  bool is_customize_z2;
  combines_x_x_t customize_rank2_z1_add_z2;
  //分频频率
	unsigned char Frequency,_Frequency_;
	LADRC_quadratic(TD_quadratic td = TD_quadratic(1,0,0),
                  float b = 10000,
                  float wc = 0,
                  float w0 = 0,
                  float max_u = 0,
                  float h = 0.001,
                  float max_z2 = 0              
                  )
		:td(td),b(b),max_u(max_u),h(h),wc(wc),w0(w0),max_z2(max_z2)
	{
    this->is_separate_wc = this->is_separate_w0 = false;//分离
		this->td.h = h;
    // if(wc != 0 || w0 != 0)
    // {
    //   return;
    // }
    // /*
    // 通常
    // wc = 10 / h
    // w0 = 4 * wc
    // */
    // this->wc = 10.0 / h;
    // this->w0 = 4.0 * this->wc;
    // /*
    // 测试过程中发现太大了所以*0.005
    // */
    // this->wc *= 0.005;
    // this->w0 *= 0.005;
  }
  //二阶线性扩张状态观测器（ESO）
  void eso_quadratic(float feedback,unsigned char rank = 2)
  {
    /*
    观测器带宽w0
    观测器带宽w0影响ESO的收敛速度。
    w0越大，z1跟踪输入信号速度越快、z2跟踪输入信号微分的速度越快、z3对扰动估计的滞后也就越小。
    w0过大，易导致观测器振荡（跟踪振荡）、观测器发散（跟踪发散）、系统产生高频噪声（z2和z3含高频噪声）；
    w0过小，会导致观测器跟踪效果变差（z2跟踪滞后）、观测器振荡（跟踪振荡）、扰动估计滞后较高（跟踪滞后，这个通过肉眼可能看不太出来）。
    一阶
    z1观测位置，β1越大收敛越快
    z2扰动量,β2越大扰动量收敛越快
    */
//    if(feedback > 8191 * 0.5)feedback -= 8191;
//    if(feedback < -8191 * 0.5)feedback += 8191;
    this->feedback = feedback;
    //β计算
    switch (rank)
    {
    case 1:
      {
        if(this->is_separate_w0 == false)
        {
          //β计算
          this->bt1 = 2.0 * this->w0;//β1
          this->bt2 = this->w0 * this->w0;//β2
        }
        //目标与观测误差
        this->e = this->feedback - this->z1;
        this->z1 += (this->z2 + bt1 * this->e + this->b * this->u) * this->h;
        this->z2 += bt2 * this->e * this->h;
      }
      break;
    case 2:
      {
        if(this->is_separate_w0 == false)
        {
          //β计算
          this->bt1 = 3.0 * this->w0;//β1
          this->bt2 = 3.0 * this->w0 * this->w0;//β2
          this->bt3 = this->w0 * this->w0 * this->w0;//β3
        }
        //目标与观测误差        
        this->e = this->feedback - this->z1;
        this->z1 += (this->z2 + bt1 * this->e) * this->h;
        if(this->is_customize_z2 != 0)
        {
          this->e2 = this->feedback_z2 - this->z2;
          this->z2 += (this->z3 + bt2 * this->e2 + this->b * this->u) * this->h;
        }
        else
        {
          this->z2 += (this->z3 + bt2 * this->e + this->b * this->u) * this->h;
        }        
        this->z3 += bt3 * this->e * this->h;
      }
      break;
    default:
      break;
    }
  }
  void set_is_separate_wc(float kp = 0,float kd = 0)
  {
    this->is_separate_wc = true;
    this->kp = kp;
    this->kd = kd;
  }
  void set_is_separate_w0(float bt1 = 0,float bt2 = 0,float bt3 = 0)
  {
    this->is_separate_w0 = true;
    this->bt1 = bt1;
    this->bt2 = bt2;
    this->bt3 = bt3;
  }
  void set_customize_rank1_z1_dt(float customize_rank1_z1_dt)
  {
    this->customize_rank1_z1_dt = customize_rank1_z1_dt;
  }
  void set_customize_z2(float feedback_z2)
  {
    this->is_customize_z2 = true;
    this->feedback_z2 = feedback_z2;
  }
  //状态误差反馈控制律（SEF）
  void sef_controller(unsigned char rank = 2)
  {
    /*
    控制器增益wc
    针对kp
    kp越大，系统响应速度越快，过渡过程越短。
    kp过大，会导致系统振荡次数增多，甚至引起输出发散；
    针对kd
    增大kd，可以抑制过渡过程中出现的超调，改善系统动态性能。
    kd过大，也会使响应提前制动，导致调节时间变长。
    b0
    b0是ADRC中唯一与被控对象有关的变量。
    当被控对象的模型未知的情况下，将其作为一个参数来整定。
    b0相当于系统总扰动的补偿系数，其决定了总扰动的估计值的变化范围、补偿分量大小。
    */
    //控制律计算
    float u0 = 0;
    switch (rank)
    {
    case 1:
      {  
        if(this->is_separate_wc == false)
        {    
          this->kp = this->wc;
        }        
        u0 = this->kp * (this->td.x1 - this->z1) - this->kd * this->customize_rank1_z1_dt - this->z2;
      }
      break;
    case 2:
      {
        if(this->is_separate_wc == false)
        {
          this->kp = this->wc * this->wc;
          this->kd = 2.0 * this->wc;
        }
        u0 = this->kp * (this->td.x1 - this->z1) + this->kd * (this->td.x2 - (this->z2)) - this->z3;
      }
      break;
    default:
      break;
    }
    //分频比较
	  if(this->Frequency > this->_Frequency_){this->_Frequency_++;return ;}
	  //分频计数
	  this->_Frequency_ = 0;
    //系统b参数
    this->u = u0 / this->b;					 
    //限幅
    if(this->u > this->max_u)this->u = this->max_u;
    if(this->u < -this->max_u)this->u = -this->max_u;
  }
  float up_data(float target,float feedback,unsigned char rank = 1,bool is_open_td = true)
  {
    this->td.td_quadratic(target,is_open_td);
    this->eso_quadratic(feedback,rank);
    this->sef_controller(rank);
    return this->u;
  }
};

//扩展状态观测器反推
struct ESO_QUADRATIC
{
	float e,z1,z2,z3,bt1,bt2,bt3,h;
	ESO_QUADRATIC(float bt1,float bt2,float bt3,float h = 0.001)
		:bt1(bt1),bt2(bt2),bt3(bt3),h(h)
	{
	}
	void up_data_z1(float feedback)//输入是z2
	{
		this->e = feedback - this->z1;
		this->z1 += (this->z2 + this->bt1 * this->e) * this->h; 
		this->z2 += (this->z3 + this->bt2 * this->e) * this->h;        
		this->z3 += (this->bt3 * this->e) * this->h;	
	}
	void up_data_z2(float feedback)//输入是z2
	{
		this->e = feedback - this->z2;
		this->z1 += (this->z2 + this->bt1 * this->e) * this->h; 
		this->z2 += (this->z3 + this->bt2 * this->e) * this->h;        
		this->z3 += (this->bt3 * this->e) * this->h;	
	}
	void up_data_z3(float feedback)//输入是z3
	{
		this->e = feedback - this->z3;
		this->z1 += (this->z2 + this->bt1 * this->e) * this->h; 
		this->z2 += (this->z3 + this->bt2 * this->e) * this->h;        
		this->z3 += (this->bt3 * this->e) * this->h;			
	}
	void clear()//清理数据
	{
		this->z1 = this->z2 = this->z3 = 0;
	}
};
