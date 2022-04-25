
#include"costmap_2d/slope_layer_cluster.h"
 #include<algorithm>

// #define k 2  //聚类数，可在kmeans算法处作参数
int k = 2;

using namespace std;

float Slopeclouds::Dist(float t1,float t2)//欧式距离
{
	return (t1 - t2)*(t1 - t2); //如何计算能让收敛的快一些
}
 
//决定该样本属于哪一个聚类，传入的是聚类的质心(也是一个组，看作x，y)和一个样本，返回的是label;
int Slopeclouds::clusterofTuple(float means[],float t)
{
	float distance = Dist(means[0],t); //这里也有问题，为什么只取第一个值
	int label = 0;
	for (int i = 0; i < k; i++)
	{
		if (Dist(means[i],t)<= distance)  
		{
			label = i;
			distance = Dist(means[i],t);
		}
	}
	return label; //找最近质心
 
}
 
//获得蔟集的平方误差,用来判断是否还需要继续迭代，传入的是蔟集的质心，以及所有归类好的样本,装着每个蔟集的容器数组，计算该聚类到自己质心的距离，所有距离的加和，返回所有的平方误差
float Slopeclouds::getVar(float means[],vector<Slopeclouds*> cluster[])
{
	float var = 0;
	for (int i = 0; i < k; i++)
	{
		vector<Slopeclouds*> t = cluster[i];
		for (int j = 0; j < t.size(); j++)
		{
			var += Dist(means[i], (*t[j]).slope);
		}
	}
	return var;
}

float Slopeclouds::ads(float oldvar,float newvar)
{ 
	 //计算平方差变化
	float p;
	p=abs(newvar-oldvar);
	return p;
}

//计算当前蔟集的质心，输入的是一个蔟集的容器，质心的计算就是对于两个属性累加后除以个数求平均，然后返回质心，所以也要初始化一个质心Tuple t
float Slopeclouds::getMeans(vector<Slopeclouds*> cluster)
{
	float t;
	int num = cluster.size();
	float meanX = 0;
	for (int i = 0; i < num; i++)
	{
		meanX += (*cluster[i]).slope;
	}
	t = meanX / num;
	return t;
}
 
vector<Slopeclouds*>  Slopeclouds::Kmeans(vector<Slopeclouds*>& slopeclouds)  //kmeans算法
{   
	//排序
	sort(slopeclouds.begin() , slopeclouds.end() ,Compare());
	// cout<<"斜率由小到大排序后："<<endl;
	// for(int i = 0 ; i<slopeclouds.size() ; i++)
	// {
	//     cout<<(*slopeclouds[i]).slope<<"   ";
	// }
	// cout<<endl;
	
	//定义与初始化
    //首先是要定义一个放置分好的蔟，那就是容器组咯，一个容器放一个蔟
    //然后还要有放k个质心的数组
	
	vector<Slopeclouds*> cluster[k];
	//容器组
	float means[k];//放k个质心的数组

	//首先设置默认的质心;  默认质心设置的有问题  
	for(int i = 0; i < k; i++)
	{
		means[i] = (*slopeclouds[i]).slope;
	}
 
	//第一次计算距离，进行分类，得到第一次的类标，容器的话是直接用push_back放置进去
	int label = 0;
	for (int i = 0; i < slopeclouds.size(); i++)
	{
		label = clusterofTuple(means, (*slopeclouds[i]).slope);
		cluster[label].push_back(slopeclouds[i]);  //容器组,一个容器放一个蔟
	}
 
    //原始的蔟
	// vector<Slopeclouds*> original;
	// for (int i = 0; i < k; i++)
	// {
	// 	vector<Slopeclouds*> t = cluster[i];
	// 	for (int j = 0; j<t.size(); j++)
	// 	{
	// 		original.push_back(t[j]);
	// 	}
	// }
    
	//如果原始簇中本身的斜率差别不大  就不用分类
	float mean = (means[0] + means[1])/2;
	vector<Slopeclouds*> tureslope;
	for(int i = 0 ; i < slopeclouds.size() ; i++)
	{
		if(abs((*slopeclouds[i]).slope - mean) < 0.1)
		{
           tureslope.push_back(slopeclouds[i]);
		}
	}

	if(tureslope.size() == slopeclouds.size())
	{
		// cout<<"是斜坡"<<endl;
		return tureslope;
	}


	float oldvar = 0;//上一轮平方差
    float newvar = getVar(means,cluster); //输入质心数组与容器组  所有点离质心的距离平方和

	//循环迭代
	int index = 1;

	if(slopeclouds.size() ==2)  //ads(oldvar ,newvar)一定等于0  newvar一定等于0
	{
		// cout<<"每类中只有一个slope ，分别是："<<endl;
		// cout<<(*slopeclouds[0]).slope<<"      "<<(*slopeclouds[1]).slope<<endl;
		if(abs((*slopeclouds[0]).slope - (*slopeclouds[1]).slope) > 0.1)
		{
			// cout<<"没有斜坡   slope 分别是 ："<<endl;
			// for(int i = 0 ; i<slopeclouds.size() ; i++)
		    // {
	     	// 	cout<<(*slopeclouds[i]).slope<<"  ";
     		// }
			// cout<<endl;
			slopeclouds.clear();
			return slopeclouds;
		}
		else
		{
			// cout<<"暂时判断为斜坡  slope 分别是 ："<<endl;
			// for(int i = 0 ; i<slopeclouds.size() ; i++)
		    // {
	     	// 	cout<<(*slopeclouds[i]).slope<<"  ";
     		// }
			//  cout<<endl;
			return slopeclouds;
		}
	}
    
	else//至少有一类中有两个slope点
	{   
		//1、聚类
        while (ads(oldvar ,newvar) >0.01)  //结束条件,可修改  两次不是很大变化就说明分类成功
		{   
			// std::cout<<"迭代 "<<std::to_string(index)<<"  次"<<std::endl;
			oldvar = newvar; 
			
			//1先计算新的k个质心
			for (int i = 0; i < k; i++)
			{
				means[i] = getMeans(cluster[i]);   //质心局限在第一次聚类后的容器中 ， 如果第一次聚类失误，后续不可能成功 ， 所以要排序，，但还是不足（预处理还是很重要， 除去明显的噪声点）
			}
			//2清空分号蔟的容器，待会才可以根据新的质心重新分配
			for (int i = 0; i < k; i++)
			{
				cluster[i].clear();
			}
			//3根据新的质心，对于原来传入的所有数据重新分配
			for (int i = 0; i < slopeclouds.size(); i++)
			{
				label = clusterofTuple(means, (*slopeclouds[i]).slope);
				cluster[label].push_back(slopeclouds[i]);
			}
			// 4最后输出
			// for (int i = 0; i < k; i++)
			// {
			// 	vector<Slopeclouds*> t = cluster[i];
			// 	cout<<"第  "<<i<<"   类元素  :"<<endl;
			// 	for (int j = 0; j < t.size(); j++)
			// 	{
			// 		cout <<(*t[j]).slope<< endl;
			// 	}
			// }
			newvar = getVar(means,cluster);
			// cout<<"oldvar : "<<oldvar<<endl;
			// cout<<"newvar : "<<newvar<<endl;
			// cout<<ads(oldvar,newvar)<<endl;
        index++;
		}

		//现在至少有一个类里面是有两条直线的，两个slope ，有两条平行的直线可以视为斜坡
		//2、加限制条件
		for (int i = 0; i < k; i++)
		{   
			vector<Slopeclouds*> t = cluster[i];
			// cout<<"第  "<<i<<"  个类里有  "<<t.size()<<"  个slope"<<endl; 
			// for (int j = 0; j < t.size(); j++)
			// {
			// 	cout <<(*t[j]).slope<<"  ";
			// }
			// cout<<endl;
			if(t.size()>1)
			{
				if(t.size() ==2)
				{
					if(abs((*t[0]).slope-(*t[1]).slope) < 0.1)
					{
						// cout<<"暂定是斜坡"<<endl;
						return t;
					}
				}
				//如果是3个以上
				//做一个差值数组（遍历t中的所有点作任意差值，取最小值，大于阈值的清除t）行不通
				//先除去边缘点（各个slope点与中位数作差，或者是与中间的两个数的均值作差）
				//只需要某一范围内的斜率，斜坡斜率
                else
				{
				vector<Slopeclouds*> tureslope1;
				// cout<<" 除去离谱点之后的容量为 : "<<t.size()<<endl;
				for(vector<Slopeclouds*>::iterator  it = t.begin(); it != t.end() ; it++) //去除最离谱的点   //但是这是从开端开始的
			    {   
					if(t.size()%2 !=0)
					{
                        int index = (t.size()-1)/2;
						if(abs((*it)->slope - t[index]->slope) < 0.1)//证明这个slope太偏了
						{
							tureslope1.push_back(*it);
						}
					}
					else
					{
						int index1 = t.size()/2;
						int index2 = t.size()/2 -1;
						float mean = (t[index1]->slope + t[index2]->slope)/2;
						if(abs((*it)->slope - mean) < 0.1)
						{
							tureslope1.push_back(*it);
						}
					}
		     	}
				// cout<<" 除去离谱点之后的容量为 : "<<tureslope1.size()<<endl;
                
				int i = 1;
                //查看斜率的范围  圈定在（-0.35 - 0.35之间）
				vector<Slopeclouds*> tureslope2;
				// cout<<" 限制斜率范围前的容量为 : "<<tureslope1.size()<<endl;
				for(vector<Slopeclouds*>::iterator  it =tureslope1.begin(); it != tureslope1.end() ; it++)     //再建一个容器，不用这个函数
				{   
					if( (*it)->slope > -0.6 && (*it)->slope < 0.6)
					{
						tureslope2.push_back(*it);
					}
				}
                // cout<<" 限制斜率范围后的容量为 : "<<tureslope2.size()<<endl;
				if(tureslope2.size() >=2)
				{
					// cout<<"暂定是一个斜坡"<<endl;
					return tureslope2;
				}

				}
			}
		}

        // cout<<"没有斜坡"<<endl;
		slopeclouds.clear();
		return slopeclouds;
	}
		
}