/*
 * puzzle.cpp
 *
 *  Created on: Feb 19, 2019
 *      Author: savvy
 */


#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>
using namespace std;

int nodes_Goal[3][3] = {{1, 2, 3},{4, 5, 6}, {7, 8, 0}};
int row[] = { 1, 0, -1, 0 };
int col[] = { 0, -1, 0, 1 };
int num=0,node_goal_num=-1;
struct Node
{

    Node* parent;
    int num;


    int mat[3][3];


    int x, y;


    int level;
};
Node* goalNode;
void storeMatrixintoTxtFile(string filename, int arr[][3])
{

	 ofstream file;
	     file.open (filename+".txt");
	     file << filename+" array is:\n";

	     std::ostringstream os;
	     	for (int i=0;i< 3;i++) {
	     		for (int j=0;j< 3;j++) {
	     		os << arr[i][j];
	     		}
	     	}

	     	std::string str(os.str());
	     	 file << str;
	     	     file.close();
}
void clearTxtFile(string filename)
{
	std::ofstream ofs;
	ofs.open(filename+".txt", std::ofstream::out | std::ofstream::trunc);
	ofs.close();
}
void appendMatrixtoTxtFile(string filename, int arr[][3])
{
	ofstream file;
	file.open (filename+".txt", std::ios_base::app);


	 std::ostringstream os;
	 	     	for (int i=0;i< 3;i++) {
	 	     		for (int j=0;j< 3;j++) {
	 	     		os << arr[i][j]<<"\t";
	 	     		}
	 	     	}

	 	     	std::string str(os.str());
	 	     	 file << str<<"\n\n";

	  file.close();
}
void appendMatrixtoTxtFile(string filename, int arr[3])
{
	ofstream file;
	file.open (filename+".txt", std::ios_base::app);

	 //file << " more lorem ipsum";
	 std::ostringstream os;
	 	     	for (int i=0;i< 3;i++) {

	 	     		os << arr[i]<<"\t\t\t";

	 	     	}

	 	     	std::string str(os.str());
	 	     	 file << str<<"\n\n";

	  file.close();
}
void formatMatrix(string type,int arr[3][3])
{
	 cout<<"The "<<type<<" node matrix is:"<<endl;

	    for(int i=0;i<3;i++)
	        {
	    	cout<<"\t"<<"-------------------------------------------------"<<endl;
	    	cout<<"\t"<<"|";
	        	for(int j=0;j<3;j++)
	        	{

	        		cout<<"\t"<<arr[i][j]<<"\t"<<"|";
	        	}
	        	cout<<endl;

	        }
	    cout<<"\t"<<"-------------------------------------------------"<<endl;

}
void printPath(Node* root)
{
    if (root == NULL)
    	return;

    printPath(root->parent);
    formatMatrix("next",root->mat);
    appendMatrixtoTxtFile("nodesPath",root->mat);
    printf("\n");
    printf("\t\t\t\t");printf("|"); printf("\n");  printf("\t\t\t\t");printf("V");
    printf("\n");
}
int goal_Check(Node* newNode)
{
	int flag=1;



	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			if(newNode->mat[i][j]!=nodes_Goal[i][j])
			{
				flag=0;
				break;
			}
		}

	}

	return flag;

}
int getInvCount(int arr[])
{
    int inv_count = 0;
    for (int i = 0; i < 9 - 1; i++)
        for (int j = i+1; j < 9; j++)
             if (arr[j] && arr[i] &&  arr[i] > arr[j])
                  inv_count++;
    return inv_count;
}
bool isSolvable(int puzzle[3][3])
{


    int invCount = getInvCount((int *)puzzle);


    return (invCount%2 == 0);
}
int alreadyVisited(vector<Node*> nodes, Node* arr)
{
	int flag=-1;
	Node* temp_node;
	for (int i = 0; i < nodes.size(); i++)
		       {
			flag=-1;
			temp_node=nodes[i];
		    	   for (int j = 0; j < 3; j++)
		           {
		    		   for (int k = 0; k < 3; k++)
		    		   {
						   if((arr->mat[j][k]==temp_node->mat[j][k]) && flag!=0)
						   {
							   flag=1;

						   }
						   else
							   flag=0;
		    		   }
		           }
		    	   if(flag==1)
		    		   break;
		       }
	if(flag==-1)
		flag=0;

		return flag;
}
Node* addNode(int mat[3][3], int x, int y, int newX,
              int newY, int level, Node* parent, vector<Node*> nodes)
{

	int arr[3];
    Node* node = new Node;
	node->num=num;
    // set pointer for path to root
    node->parent = parent;

    // copy data from parent node to current node
    memcpy(node->mat, mat, sizeof node->mat);

    // move tile by 1 postion
    swap(node->mat[x][y], node->mat[newX][newY]);
    // set number of moves so far
    node->level = level;

    // update new blank tile cordinates
    node->x = newX;
    node->y = newY;


if(!alreadyVisited(nodes,node))
    {

	 node->num=num++;
	 arr[0]=node->num;
	 if(node->parent==NULL)
	     arr[1]=-1;
	 else
	     arr[1]=node->parent->num;
	 arr[2]=level;
	 if(goal_Check(node))
		 {
		 node_goal_num=node->num;
		 goalNode=node;
		 }

	appendMatrixtoTxtFile("nodes",node->mat);
	appendMatrixtoTxtFile("nodesInfo",arr);
	return node;
    }
else
	return NULL;
}
int canMove(int x, int y)
{
    return (x >= 0 && x < 3 && y >= 0 && y < 3);
}
void blankTilePosition(int &i,int &j, Node* node)
{
	for(int k=0;k<3;k++)
	{
		for(int l=0;l<3;l++)
		{
			if(node->mat[k][l]==0)
			{
				i=k;
				j=l;
			}
		}
	}
}

void processNodes(Node* node, vector<Node*> nodes,int nodes_Goal[3][3])
{

	Node* temp_node=node;
	int cur_pos_i=-1,cur_pos_j=-1;

    vector<Node*>myqueue;


		int flag=1;
			while(flag==1 && node_goal_num==-1)
			{


							blankTilePosition(cur_pos_i,cur_pos_j,temp_node);
							for (int i = 0; i < 4; i++)
							        {
							            if (canMove(cur_pos_i + row[i], cur_pos_j + col[i]))
							            {
							               Node* child = addNode(temp_node->mat, temp_node->x,
							                		temp_node->y, temp_node->x + row[i],
							                		temp_node->y + col[i],
							                              temp_node->level + 1, temp_node, nodes);

							                if(child!=NULL)
							                {
							                nodes.push_back(child);
							                myqueue.push_back(child);
							                }
							                if(node_goal_num!=-1)
							                	{
							                	break;
							                	}

							            }
							        }

								temp_node=myqueue[0];
                                myqueue.erase(myqueue.begin(),myqueue.begin()+1);
                                if(myqueue.size()<=0)
                                    {
                                        flag=0;
                                        break;
                                    }

			}
			if(flag==0)
				cout<<"GOAL CANNOT BE REACHED!!!! Computations:"<<nodes.size()<<endl;
			else
				    {
				    	cout<<"GOAL MATCHED!!!! Computations:"<<nodes.size()<<endl;
				    	 printPath(goalNode);
				    }




}




string numCheck(int input, int arr[])
{
	int k=-1;

	for(int l=0;l<3;l++)
	    {

		for(int j=0;j<3;j++)
		    	{
			k++;
	    				if(input==arr[k])
	    				{

	    					string s = std::to_string(l);
	    					string m = std::to_string(j);
	    					return std::string(s+','+m);
	    				}

	    			}
	    	}

	return string("0");
}





void swap(int &a,int &b){
	int temp;
		temp = a;
	    a = b;
	    b = temp;
}




int main(){
  //  int num1=5 , num2=10, temp;
    int temp[9]= { -1, -1, -1,-1, -1, -1, -1, -1, -1},k=0, input,flag,valid,nodes_init[3][3];
int X0,Y0;
    vector<vector<int> >  nodes_init_v;
    vector<Node*>nodes;
    //vector<int> node_temp2D;
    clearTxtFile("nodes");
    clearTxtFile("nodesPath");
    //clearTxtFile("nodesInfo");
    std::ofstream file;
    file.open("nodesInfo.txt", std::ofstream::out | std::ofstream::trunc);
    file <<"Node #\tParent #\tCost2Come\n\n";
    file.close();




    string ans = "null";
    cout<<"Enter a 3X3 dimensional array."<<endl;

    for(int i=0;i<3;i++)
    {
    	for(int j=0;j<3;j++)
    	{
    		valid=0;
    		cout << "Enter a number between 0-8."<< " : "<<endl;
    		do{


    		cin >>input;
    		if(input>=0 && input <9)
    			valid =1;
    		else
    			cout << "Please enter a valid number between 0-8:"<<endl;
    		}while(valid==0);
    		if(i==0 && j==0)
			{
		    	temp[k++]=input;
		    	nodes_init[i][j]=input;
		    }
    		else
		    {
			do{
				flag=0;
					ans=numCheck(input,temp);
					if(ans!="0")
					{
						cout << "This number has already been entered at position : "<<ans<<endl;
						cout<<"Please enter another number between 0-8."<<endl;
						cin >>input;
					}
					else
						flag=1;
				}
			while(flag==0);
			if(ans=="0")
			{
				temp[k++]=input;
				nodes_init[i][j]=input;
				if(input==0)
				{
					X0=i;
					Y0=j;
				}
			}

		    }
    	}
    }
   formatMatrix("User input",nodes_init);
   if(isSolvable(nodes_init)==0)
   cout<<"GOAL CANNOT BE REACHED!!!!"<<endl;
   else
    {
	    Node* nodes_init_node = addNode(nodes_init, X0,Y0, X0,Y0,0, NULL,nodes);
        nodes.push_back(nodes_init_node);
        processNodes(nodes_init_node,nodes,nodes_Goal);
    }
   formatMatrix("Final",nodes_Goal);

    return 0;
}
