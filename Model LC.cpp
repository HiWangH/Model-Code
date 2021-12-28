
#include <ilcplex/ilocplex.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace std;
ILOSTLBEGIN
const IloInt M=10000;// Sufficiently large positive constant
const IloNum q1=10,q2=1;//Parameters of the objective
const IloNum T=16;//Planning horizon
const IloInt Trailer_Num=2;//K
const IloInt speed=80;

int main(int argc,char** argv)
{
	clock_t BeginTime1=clock();
	ofstream outfile;
	fstream fs;
	outfile.open("result1.txt");
	char szFileName[20];
try
	{
	outfile<<'\t'<<"Vehicles"<<'\t'<<"Vehicles costs"<<'\t'<<"Duration time (h)"<<'\t'<<"Objective (h)"<<'\t'<<"Traveling time (h)"<<'\t'<<"Waiting time (h)"<<'\t'<<"CPU (s)"<<endl;
	for(int A=1;A<25;A++)
	{ 
	IloEnv env;//Cplex environment
	int gh=0;
	float data=0;
	vector<float> tempory_vector;
	vector<double> tempory_X(1,100);
	vector<double> tempory_Y(1,100);
	string line;
	sprintf(szFileName,"datafile%d.txt",A);
	fs.open(szFileName,ios_base::in);
  	if(fs.fail()){fs.close();cin.get(); cin.get();gh=1;}
    else 	{while(!fs.eof())
				{
			getline(fs,line);
			if(line=="# of pickup customers"||line=="# of delivery customers"||line=="# of customers"||line=="# of tasks"||line=="# of given tractors"||line=="c1"||line=="c2"||line=="c3"||line=="timeperiod"||line=="package time"||line=="terminal coordinates"||line=="customer coordinate"||line=="distance matrix"||line=="ENDDATA")
           continue;
			else  {istringstream iss(line);iss>>data;tempory_vector.push_back(data);}
				}
			}
	  if(gh==1) continue;
	  fs.close();
	IloInt JiYiE;//End of Phase I suborder of outbound orders
    IloInt JiErE;//End of Phase II suborder of outbound orders
	IloInt Customer=0;//Customer number
	vector< vector<float> > tt;
	vector<float> pp;//Container packing / unpacking time
	vector< vector<float> > TravelTime;
	Customer=tempory_vector[2];
	const IloInt TotalNode=2*Customer+1;
	const IloInt Medial=(TotalNode-1)/2;
	const IloInt SecondStageB=(TotalNode-1)/2+1;
	JiYiE=tempory_vector[0]+1;
	JiErE=JiYiE+Medial;
	for(int i=9;i<9+Customer;i++)
		pp.push_back(tempory_vector[i]);
	for(int i=0;i<pp.size();i++) pp[i]=(int)(pp[i]*1000+0.5)/1000.0;
	int Cooridinate_begin=11+Customer;
	int Cooridinate_end=Cooridinate_begin+2*Customer;
	int Cooridinate_middle=Cooridinate_begin+Customer;
	for(int i=Cooridinate_begin;i<Cooridinate_middle;i++)	tempory_X.push_back(tempory_vector[i]);
	for(int i=Cooridinate_middle;i<Cooridinate_end;i++)	tempory_Y.push_back(tempory_vector[i]);
	int size_vec=tempory_X.size();
	vector<float> vee;
	for(int i=0;i<size_vec;i++)
	{vee.clear();
	for(int j=0;j<size_vec;j++)
		{
		double dis=sqrt((tempory_X[i]-tempory_X[j])*(tempory_X[i]-tempory_X[j])+(tempory_Y[i]-tempory_Y[j])*(tempory_Y[i]-tempory_Y[j]));
			vee.push_back(dis/speed);			
		}
	 tt.push_back(vee);
	}


	for(int i=0;i<TotalNode;i++)
	{vee.clear();
	for(int j=0;j<TotalNode;j++) vee.push_back(0);
	TravelTime.push_back(vee);
	}

	for(int i=SecondStageB;i<TotalNode;i++)
	{
		TravelTime[i][0]=TravelTime[i-Medial][0]=TravelTime[0][i]=TravelTime[0][i-Medial]=tt[0][i-Medial];
	}
	for(int i=1;i<TotalNode;i++)
	 {for(int j=1;j<SecondStageB;j++)
		{
			if(i<SecondStageB)
				{TravelTime[i][j]=TravelTime[i][j+Medial]=tt[i][j];}
		else{TravelTime[i][j]=TravelTime[i][j+Medial]=tt[i-Medial][j];}
			
		}
	 }

		 for(int i=1;i<TotalNode;i++)
	 {for(int j=1;j<TotalNode;j++)
		{if(i<SecondStageB){if(i==j)TravelTime[i][j]=2*tt[0][i];
	                     if(j==i+Medial)TravelTime[i][j]=0;
						}
		else {if(i==j)TravelTime[i][j]=2*tt[0][i-Medial];
	                     if(j==i-Medial)TravelTime[i][j]=M;
			  }
		 }
	 }

	for(int i=0;i<TotalNode;i++)
		for(int j=0;j<TotalNode;j++) TravelTime[i][j]=(int)(TravelTime[i][j]*1000+0.5)/1000.0;
	


	clock_t BeginTime=clock();
	IloModel model1(env);//Define model
	IloArray<IloBoolVarArray> X1(env,TotalNode);//2D boolean decision variables
	IloArray<IloIntVarArray> Y1I(env,TotalNode);//2D int decision variables for IFT
	IloArray<IloIntVarArray> Y1E(env,TotalNode);//2D int decision variables for OFT
	IloArray<IloIntVarArray> Z1(env,TotalNode);//2D int decision variables for ET
	
	IloArray<IloNumVarArray> traveltime(env,TotalNode);//Transfer time
	IloNumVarArray S1(env,TotalNode,0,T);//Service starting time
	

	for(int k=0;k<TotalNode;k++)
	{X1[k]=IloBoolVarArray(env,TotalNode);//initialing all the decison variables
	Y1I[k]=IloIntVarArray(env,TotalNode,0,Trailer_Num);
    Y1E[k]=IloIntVarArray(env,TotalNode,0,Trailer_Num);
	Z1[k]=IloIntVarArray(env,TotalNode,0,Trailer_Num);
	traveltime[k]=IloNumVarArray(env,TotalNode,0.0,T);
	}

	for(int i=1;i<TotalNode;i++)//Strengthened Sub-tour Elimination Inequality
		for(int j=1;j<TotalNode;j++)
			if(i!=j) model1.add(X1[i][j]+X1[j][i]<=1);

	for(int i=1;i<SecondStageB;i++)//Strengthened Deadlock Elimination Inequality
		for(int j=1;j<SecondStageB;j++)
			if(i!=j) model1.add(X1[i+Medial][j]+X1[j+Medial][i]<=1);

	IloNumVarArray S_mix1(env,TotalNode,0,M/2);
	IloNumVarArray S_mix2(env,TotalNode,0,M/2);
	IloNumExpr OBJ_temp(env);
	IloNumExpr temp0c(env);
	for(int j=1;j<TotalNode;j++)
			{temp0c +=q1*X1[0][j];
			}
	IloNumExpr temp1c(env);
	IloNumExpr duration1(env);
    for(int i=1;i<TotalNode;i++)		
			    duration1 +=q2*S_mix1[i];
	IloNumExpr duration11(env);
    for(int i=1;i<TotalNode;i++)		
			    duration11 +=q2*S_mix2[i];
	temp1c=duration1-duration11;
	OBJ_temp=temp0c+temp1c;
	model1.add(IloMinimize(env,OBJ_temp));
	for(int i=1;i<TotalNode;i++)
	{
		model1.add(S_mix1[i]-S1[i]>= -T*(1-X1[i][0])+TravelTime[i][0]);
		model1.add(S_mix2[i]<=T*X1[0][i]);
		model1.add(S_mix2[i]<= T*(1-X1[0][i])-TravelTime[0][i]+S1[i]);
	}
 	IloNumExpr totaltraveltime(env);//Working Time Inequality
	for(int i=0;i<TotalNode;i++)
    for(int j=0;j<TotalNode;j++)
		if(i!=j) totaltraveltime+=TravelTime[i][j]*X1[i][j];
    model1.add(duration1-duration11>=totaltraveltime);

	for(int i=1;i<TotalNode;i++)
		{model1.add(traveltime[0][i]==TravelTime[0][i]);
	     model1.add(traveltime[i][0]==TravelTime[i][0]);
	    }

	for(int j=1;j<TotalNode;j++)
	{IloIntExpr temp2c(env);
		{for(int i=0;i<TotalNode;i++)
			{
				if(i!=j) temp2c +=X1[i][j];
			}
		}
	model1.add(temp2c==1);//Constraint (2)
	}

	for(int j=1;j<TotalNode;j++)
		{IloIntExpr temp3c(env);
		IloIntExpr temp4c(env);
			 for(int i=0;i<TotalNode;i++)
				{if(i!=j) 
					{temp3c +=X1[i][j];
			         temp4c +=X1[j][i];
					}
				}
			 model1.add(temp3c==temp4c);//Constraint (3)
		}


   for(int i=1;i<TotalNode;i++)
		{for(int j=1;j<TotalNode;j++)
			{
			    {
					if(i!=j)
					{IloNumExpr temp10c(env);
					 temp10c=S1[i]+traveltime[i][j]-M*(1-X1[i][j])-S1[j];//Constraint (4)
					model1.add(temp10c<=0);
					}
                }
			}
		}
   for(int q=1;q<SecondStageB;q++)
		{
		model1.add(S1[q]+pp[q-1]<=S1[q+Medial]);//Constraint (5)
		model1.add(S1[q]>=TravelTime[0][q]);
		}

   for(int i=SecondStageB;i<TotalNode;i++)
		{model1.add(S1[i]+TravelTime[i][0]<=T);
        }


   for(int i=0;i<TotalNode;i++)
   {for(int j=0;j<TotalNode;j++)
		{if(i!=j) model1.add(Y1E[i][j] +Y1I[i][j] +Z1[i][j]<=Trailer_Num*X1[i][j]);}//Constraint (6)
   }

   IloIntExpr trailer_yIO(env);
   for(int i=1;i<TotalNode;i++)
	trailer_yIO+=Y1I[i][0]+Y1E[0][i];
	model1.add(trailer_yIO==0);}//Constraint (7)
  
  //Constraint (54)-(57)
		for(int j=JiYiE;j<SecondStageB;j++)
		{
		for(int i=1;i<TotalNode;i++)
		{if(j!=i)
				{
		model1.add(IloIfThen(env,Y1I[i][j]>=1,traveltime[i][j]==TravelTime[i][j]));
		model1.add(IloIfThen(env,Y1I[i][j]==0,traveltime[i][j]==TravelTime[i][0]+TravelTime[0][j]));
				}
		}
		}

		for(int j=JiYiE;j<SecondStageB;j++)
		{
		IloIntExpr trailer_yI_ji(env);
		IloIntExpr trailer_yI_ij(env);
		IloIntExpr trailer_yE_ij(env);
		IloIntExpr trailer_yE_ji(env);
		IloIntExpr trailer_z_ij(env);
		IloIntExpr trailer_z_ji(env);
		for(int i=0;i<TotalNode;i++)
			{if(j!=i){
			trailer_yI_ji +=Y1I[j][i];
			trailer_yI_ij +=Y1I[i][j];
			trailer_yE_ij +=Y1E[i][j];
			trailer_yE_ji +=Y1E[j][i];
			trailer_z_ij +=Z1[i][j];
			trailer_z_ji +=Z1[j][i];
					}
		    }  
      model1.add(IloIfThen(env,trailer_yI_ij>=1,trailer_yI_ji==trailer_yI_ij-1));//
		model1.add(IloIfThen(env,trailer_yI_ij==0,trailer_yI_ji<=Trailer_Num-1));//
      model1.add(IloIfThen(env,trailer_yI_ij>=1,trailer_yE_ji==trailer_yE_ij));//
		model1.add(IloIfThen(env,trailer_yI_ij==0,trailer_yE_ji==0));//
      model1.add(IloIfThen(env,trailer_yI_ij>=1,trailer_z_ji==trailer_z_ij));//
		model1.add(IloIfThen(env,trailer_yI_ij==0,trailer_z_ji<=Trailer_Num-1));//
		model1.add(IloIfThen(env,trailer_yI_ij==0,trailer_z_ji+trailer_yI_ji<=Trailer_Num-1));//
		}
  


		for(int j=1;j<JiYiE;j++)
		{
		for(int i=1;i<TotalNode;i++)
		{if(j!=i)
				{
		model1.add(IloIfThen(env,Z1[i][j]>=1,traveltime[i][j]==TravelTime[i][j]));
		model1.add(IloIfThen(env,Z1[i][j]==0,traveltime[i][j]==TravelTime[i][0]+TravelTime[0][j]));
				}
		}
		}
		
		for(int j=1;j<JiYiE;j++)
		{

		IloIntExpr trailer_yI_ij(env);
		IloIntExpr trailer_yI_ji(env);
		IloIntExpr trailer_yE_ij(env);
		IloIntExpr trailer_yE_ji(env);
		IloIntExpr trailer_z_ji(env);
		IloIntExpr trailer_z_ij(env);
		for(int i=0;i<TotalNode;i++)
				{if(j!=i){
					trailer_yI_ij +=Y1I[i][j];
					trailer_yI_ji +=Y1I[j][i];	
					trailer_yE_ij +=Y1E[i][j];
					trailer_yE_ji +=Y1E[j][i];
					trailer_z_ji +=Z1[j][i];
					trailer_z_ij +=Z1[i][j];
						 }	
				}
        model1.add(IloIfThen(env,trailer_z_ij>=1,trailer_yI_ji==trailer_yI_ij));//
		model1.add(IloIfThen(env,trailer_z_ij==0,trailer_yI_ji<=Trailer_Num-1));//

        model1.add(IloIfThen(env,trailer_z_ij>=1,trailer_yE_ji==trailer_yE_ij));//
		model1.add(IloIfThen(env,trailer_z_ij==0,trailer_yE_ji==0));//

        model1.add(IloIfThen(env,trailer_z_ij>=1,trailer_z_ji==trailer_z_ij-1));//
		model1.add(IloIfThen(env,trailer_z_ij==0,trailer_z_ji<=Trailer_Num-1));//
		model1.add(IloIfThen(env,trailer_z_ij==0,trailer_yI_ji+trailer_z_ji<=Trailer_Num-1));//
		}


		for(int j=JiErE;j<TotalNode;j++)
		{
		for(int i=1;i<TotalNode;i++)
		{if(j!=i)
				{
		model1.add(IloIfThen(env,Z1[i][j]+Y1I[i][j]+Y1E[i][j]>=Trailer_Num,traveltime[i][j]==TravelTime[i][0]+TravelTime[0][j]));
		model1.add(IloIfThen(env,Z1[i][j]+Y1I[i][j]+Y1E[i][j]<=Trailer_Num-1,traveltime[i][j]==TravelTime[i][j]));
				}
		}
		}
		
		for(int j=JiErE;j<TotalNode;j++)
		{
		IloIntExpr trailer_yI_ij(env);
		IloIntExpr trailer_yI_ji(env);
		IloIntExpr trailer_yE_ij(env);
		IloIntExpr trailer_yE_ji(env);
		IloIntExpr trailer_z_ij(env);
		IloIntExpr trailer_z_ji(env);
		for(int i=0;i<TotalNode;i++)
				{if(j!=i){
					trailer_yI_ij +=Y1I[i][j];
					trailer_yI_ji +=Y1I[j][i];
					trailer_yE_ij +=Y1E[i][j];
					trailer_yE_ji +=Y1E[j][i];
					trailer_z_ij +=Z1[i][j];
					trailer_z_ji +=Z1[j][i];
						 }	
				}
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij>=Trailer_Num,trailer_yI_ji<=Trailer_Num-1));//
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij<=Trailer_Num-1,trailer_yI_ji==trailer_yI_ij));//

		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij>=Trailer_Num,trailer_yE_ji==0));//
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij<=Trailer_Num-1,trailer_yE_ji==trailer_yE_ij));//

		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij>=Trailer_Num,trailer_z_ji>=1));//
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij<=Trailer_Num-1,trailer_z_ji==trailer_z_ij+1));//

		}


		for(int j=SecondStageB;j<JiErE;j++)
		{for(int i=1;i<TotalNode;i++)
		{if(j!=i)
				{
		model1.add(IloIfThen(env,Z1[i][j]+Y1I[i][j]+Y1E[i][j]>=Trailer_Num,traveltime[i][j]==TravelTime[i][0]+TravelTime[0][j]));
		model1.add(IloIfThen(env,Z1[i][j]+Y1I[i][j]+Y1E[i][j]<=Trailer_Num-1,traveltime[i][j]==TravelTime[i][j]));
				}
		}
		}
		
		for(int j=SecondStageB;j<JiErE;j++)
		{
		IloIntExpr trailer_yI_ij(env);
		IloIntExpr trailer_yI_ji(env);
		IloIntExpr trailer_yE_ij(env);
		IloIntExpr trailer_yE_ji(env);
		IloIntExpr trailer_z_ij(env);
		IloIntExpr trailer_z_ji(env);
		for(int i=0;i<TotalNode;i++)
				{if(j!=i){
					trailer_yI_ij +=Y1I[i][j];
					trailer_yI_ji +=Y1I[j][i];
					trailer_yE_ij +=Y1E[i][j];
					trailer_yE_ji +=Y1E[j][i];
					trailer_z_ij +=Z1[i][j];
					trailer_z_ji +=Z1[j][i];
						 }	
				}

		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij>=Trailer_Num,trailer_yI_ji<=Trailer_Num-1));//
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij<=Trailer_Num-1,trailer_yI_ji==trailer_yI_ij));//

		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij>=Trailer_Num,trailer_yE_ji==1));//
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij<=Trailer_Num-1,trailer_yE_ji==trailer_yE_ij+1));//

		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij>=Trailer_Num,trailer_z_ji<=Trailer_Num-1));//
		model1.add(IloIfThen(env,trailer_yI_ij+trailer_yE_ij+trailer_z_ij<=Trailer_Num-1,trailer_z_ji==trailer_z_ij));//
		}


		
	IloCplex cplex1(model1);
	cplex1.setParam(IloCplex::Param::TimeLimit,3600);
	cplex1.setParam(IloCplex::Param::MIP::Strategy::NodeSelect,0);
	cplex1.setOut(env.getNullStream());
	cplex1.solve();
	if(cplex1.getStatus()==IloAlgorithm::Infeasible)
		env.out()<<"No Solution!!"<<endl;
	clock_t EndTime=clock();
	outfile<<"R"<<A<<'\t'<<cplex1.getValue(temp0c)<<'\t'<<cplex1.getValue(temp1c)<<'\t'<<cplex1.getObjValue()<<'\t'<<static_cast<double>(EndTime-BeginTime)/CLOCKS_PER_SEC<<endl;
	env.end();

    outfile.close();
	 
	}
catch(IloException &ex)
	{cerr<<"Error!!!!"<<ex<<endl;
clock_t EndTime1=clock();
cout<<static_cast<double>(EndTime1-BeginTime1)/CLOCKS_PER_SEC<<"s"<<endl;
outfile<<static_cast<double>(EndTime1-BeginTime1)/CLOCKS_PER_SEC<<"s"<<endl;
outfile.close();
	}
catch(...)
	{cerr<<"Error   error"<<endl;
	}
system("Pause");
return 0;
}
