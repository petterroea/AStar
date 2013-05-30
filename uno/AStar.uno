using Uno;
using Uno.Collections;
using Uno.Graphics;
using Uno.Audio;
using Uno.Scenes;
using Uno.Content;
using Uno.Content.Models;

namespace Petterroea.Utils
{
	/**
 		ASTAR - Lightweight A* pathfinding algoritm implementation in UNO
 		Copyright (C) 2013  Liam Svanåsbakken Crouch(Known as Liam S. Crouch internationally) aka petterroea
		You should of gotten a copy of the Licence with this source code, please see "LICENCE"
 	*/
	class AStar
	{
		/**
			Gives you a path from start to end based on a map, where only straight paths are allowed(No diagnoal)
			
			Please note that it only takes a 1-dimensional array, because that is more proffesional. Google converting a 2dimensional array to a 1dimensional array ;).
			@param map An array of bools. False is air, true is wall
			@param w Width of map
			@param h Height of map
			@param start The start int2
			@param end The end int2
			@return A list containing all int2s in the best path. If no path is found, the list will be empty.
			@throws IllegalArgumentException if one of the int2s are outside the area
		 */
		public static List<int2> getPath(bool[] map, int w, int h, int2 start, int2 end)
		{
			//First, some checking
			if(start.X>=w||start.Y>=h||start.X<0||start.Y<0)
			{
				throw new Exception("Start is outside of array!");
			}
			if(end.X>=w||end.Y>=h||end.X<0||end.Y<0)
			{
				throw new Exception("End is outside of array!");
			}
			List<int2> todoList = new List<int2>();//List of int2s to do for the current iteration
			todoList.Add(start);//Add a start int2.
			int[] numArray = new int[map.Length]; //Array containing length weight
			bool foundExit=false; //So we can detect if we have reached the end
			numArray[start.X+(start.Y*w)]=1; //Set start weight number
			while(todoList.Count>0&&foundExit==false) //While we still have int2s to scan, and we have not reached the exit
			{
				List<int2> nextIteration = new List<int2>(); //List of int2s to scan for next iteration
				for(int i = 0; i < todoList.Count; i++)//Iterate over the list
				{
					int x = todoList[i].X;
					int y = todoList[i].Y;
					if(x==end.X&&y==end.Y)
					{
						foundExit=true;
						break;
					}
					int arrayPos=x+(y*w);
					int thisNum=numArray[arrayPos];
					if(x<w-1)
					{
						if(!map[x+1+(y*w)]&&(numArray[x+1+(y*w)]>thisNum+1||numArray[x+1+(y*w)]==0))
						{
							nextIteration.Add(int2(x+1, y));
							numArray[x+1+(y*w)]=thisNum+1;
						}
					}
					if(x>0)
					{
						if(!map[x-1+(y*w)]&&(numArray[x-1+(y*w)]>thisNum+1||numArray[x-1+(y*w)]==0))
						{
							nextIteration.Add(int2(x-1, y));
							numArray[x-1+(y*w)]=thisNum+1;
						}
					}
					if(y<h-1)
					{
						if(!map[x+((y+1)*w)]&&(numArray[x+((y+1)*w)]>thisNum+1||numArray[x+((y+1)*w)]==0))
						{
							nextIteration.Add(int2(x, y+1));
							numArray[x+((y+1)*w)]=thisNum+1;
						}
					}
					if(y>0)
					{
						if(!map[x+((y-1)*w)]&&(numArray[x+((y-1)*w)]>thisNum+1||numArray[x+((y-1)*w)]==0))
						{
							nextIteration.Add(int2(x, y-1));
							numArray[x+((y-1)*w)]=thisNum+1;
						}
					}
				}
				todoList=nextIteration; //Set the todo list as the new iteration-list
			}
			List<int2> int2s = new List<int2>(); //We are done scanning, so we init a list of all int2s in the path back.
			if(foundExit)
			{
				bool gotBack = false;
				int xat=end.X;
				int yat=end.Y;
				while(!gotBack)
				{
					int2s.Add(int2(xat, yat));
	 				if(xat==start.X&&yat==start.Y) //We found the way back
					{
						gotBack=true;
						break;
					}
					int arrayPos=xat+(yat*w); //Get current position and store it in a variable, to make the code more readable.
					int thisNum=numArray[arrayPos]; //Get current weight and store it in a variable, to make the code more readable.
					int lowestNeighbour=thisNum; //Store lowest neighbour weight, so we can decide what neighbour is closest to start.
					string dir="NONE"; //Store the direction in a string, because i can.
					if(xat>0) //Only check left side if there is a spot to the left
					{
						if(numArray[xat-1+(yat*w)]<lowestNeighbour&&numArray[xat-1+(yat*w)]>0)
						{
							lowestNeighbour=numArray[xat-1+(yat*w)];
							dir="LEFT";
						}
					}
					if(yat>0) //Only check up if there is a spot upwards
					{
						if(numArray[xat+((yat-1)*w)]<lowestNeighbour&&numArray[xat+((yat-1)*w)]>0)
						{
							lowestNeighbour=numArray[xat+((yat-1)*w)];
							dir="UP";
						}
					}
					if(xat<w-1) //Only check right if there is a spot to the right
					{
						if(numArray[xat+1+(yat*w)]<lowestNeighbour&&numArray[xat+1+(yat*w)]>0)
						{
							lowestNeighbour=numArray[xat+1+(yat*w)];
							dir="RIGHT";
						}
					}
					if(yat<h-1) //Only check down if there is a spot downwards.
					{
						if(numArray[xat+((yat+1)*w)]<lowestNeighbour&&numArray[xat+((yat+1)*w)]>0)
						{
							lowestNeighbour=numArray[xat+((yat+1)*w)];
							dir="DOWN";
						}
					}
					if(dir == "LEFT") //Find the direction to do the next iteration with
					{
						xat=xat-1;
					}
					else if(dir == "RIGHT")
					{
						xat=xat+1;
					}
					else if(dir == "UP")
					{
						yat=yat-1;
					}
					else if(dir == "DOWN")
					{
						yat=yat+1;
					}
					else
					{
						//Something is wrong
					}
				}
			}
			return int2s;
		}
		/**
			Gives you a path from start to end based on a map, where diagnoal paths are allowed.
			
			Please note that it only takes a 1-dimensional array, because that is more proffesional. Google converting a 2dimensional array to a 1dimensional array ;).
			@param map An array of bools. False is air, true is wall
			@param w Width of map
			@param h Height of map
			@param start The start int2
			@param end The end int2
			@return A list containing all int2s in the best path. If no path is found, the list will be empty.
			@throws IllegalArgumentException if one of the int2s are outside the area
		 */
		public static List<int2> getPathWithDiagnoal(bool[] map, int w, int h, int2 start, int2 end)
		{
			//First, some checking
			if(start.X>=w||start.Y>=h||start.X<0||start.Y<0)
			{
				throw new Exception("Start is outside of array!");
			}
			if(end.X>=w||end.Y>=h||end.X<0||end.Y<0)
			{
				throw new Exception("End is outside of array!");
			}
			List<int2> todoList = new List<int2>();//List of int2s to do for the current iteration
			todoList.Add(start);//Add a start int2.
			int[] numArray = new int[map.Length]; //Array containing length weight
			bool foundExit=false; //So we can detect if we have reached the end
			numArray[start.X+(start.Y*w)]=1; //Set start weight number
			while(todoList.Count>0&&foundExit==false) //While we still have int2s to scan, and we have not reached the exit
			{
				List<int2> nextIteration = new List<int2>(); //List of int2s to scan for next iteration
				for(int i = 0; i < todoList.Count; i++)//Iterate over the list
				{
					int x = todoList[i].X;
					int y = todoList[i].Y;
					if(x==end.X&&y==end.Y)
					{
						foundExit=true;
						break;
					}
					int arrayPos=x+(y*w);
					int thisNum=numArray[arrayPos];
					if(x<w-1)
					{
						if(!map[x+1+(y*w)]&&(numArray[x+1+(y*w)]>thisNum+1||numArray[x+1+(y*w)]==0))
						{
							nextIteration.Add(int2(x+1, y));
							numArray[x+1+(y*w)]=thisNum+1;
						}
					}
					if(x>0)
					{
						if(!map[x-1+(y*w)]&&(numArray[x-1+(y*w)]>thisNum+1||numArray[x-1+(y*w)]==0))
						{
							nextIteration.Add(int2(x-1, y));
							numArray[x-1+(y*w)]=thisNum+1;
						}
					}
					if(y<h-1)
					{
						if(!map[x+((y+1)*w)]&&(numArray[x+((y+1)*w)]>thisNum+1||numArray[x+((y+1)*w)]==0))
						{
							nextIteration.Add(int2(x, y+1));
							numArray[x+((y+1)*w)]=thisNum+1;
						}
					}
					if(y>0)
					{
						if(!map[x+((y-1)*w)]&&(numArray[x+((y-1)*w)]>thisNum+1||numArray[x+((y-1)*w)]==0))
						{
							nextIteration.Add(int2(x, y-1));
							numArray[x+((y-1)*w)]=thisNum+1;
						}
					}
				}
				todoList=nextIteration; //Set the todo list as the new iteration-list
			}
			List<int2> int2s = new List<int2>(); //We are done scanning, so we init a list of all int2s in the path back.
			if(foundExit)
			{
				bool gotBack = false;
				int xat=end.X;
				int yat=end.Y;
				while(!gotBack)
				{
					int2s.Add(int2(xat, yat));
	 				if(xat==start.X&&yat==start.Y) //We found the way back
					{
						gotBack=true;
						break;
					}
					int arrayPos=xat+(yat*w); //Get current position and store it in a variable, to make the code more readable.
					int thisNum=numArray[arrayPos]; //Get current weight and store it in a variable, to make the code more readable.
					int lowestNeighbour=thisNum; //Store lowest neighbour weight, so we can decide what neighbour is closest to start.
					string dir="NONE"; //Store the direction in a string, because i can.
					if(xat>0) //Only check left side if there is a spot to the left
					{
						if(numArray[xat-1+(yat*w)]<lowestNeighbour&&numArray[xat-1+(yat*w)]>0)
						{
							lowestNeighbour=numArray[xat-1+(yat*w)];
							dir="LEFT";
						}
					}
					if(yat>0) //Only check up if there is a spot upwards
					{
						if(numArray[xat+((yat-1)*w)]<lowestNeighbour&&numArray[xat+((yat-1)*w)]>0)
						{
							lowestNeighbour=numArray[xat+((yat-1)*w)];
							dir="UP";
						}
					}
					if(xat<w-1) //Only check right if there is a spot to the right
					{
						if(numArray[xat+1+(yat*w)]<lowestNeighbour&&numArray[xat+1+(yat*w)]>0)
						{
							lowestNeighbour=numArray[xat+1+(yat*w)];
							dir="RIGHT";
						}
					}
					if(yat<h-1) //Only check down if there is a spot downwards.
					{
						if(numArray[xat+((yat+1)*w)]<lowestNeighbour&&numArray[xat+((yat+1)*w)]>0)
						{
							lowestNeighbour=numArray[xat+((yat+1)*w)];
							dir="DOWN";
						}
					}
					if(yat>0&&xat>0)
					{
						if(numArray[xat-1+((yat-1)*w)]<lowestNeighbour&&numArray[xat-1+((yat-1)*w)]>0&&numArray[xat+((yat-1)*w)]>0&&numArray[xat-1+((yat)*w)]>0)
						{
							lowestNeighbour=numArray[xat-1+((yat-1)*w)];
							dir="LEFTUP";
						}
					}
					if(yat>0&&xat<h-1)
					{
						if(numArray[xat+1+((yat-1)*w)]<lowestNeighbour&&numArray[xat+1+((yat-1)*w)]>0&&numArray[xat+((yat-1)*w)]>0&&numArray[xat+1+((yat)*w)]>0)
						{
							lowestNeighbour=numArray[xat+1+((yat-1)*w)];
							dir="RIGHTUP";
						}
					}
					if(yat<w-1&&xat<h-1)
					{
						if(numArray[xat+1+((yat+1)*w)]<lowestNeighbour&&numArray[xat+1+((yat+1)*w)]>0&&numArray[xat+((yat+1)*w)]>0&&numArray[xat+1+((yat)*w)]>0)
						{
							lowestNeighbour=numArray[xat+1+((yat+1)*w)];
							dir="RIGHTDOWN";
						}
					}
					if(yat<w-1&&xat>0)
					{
						if(numArray[xat-1+((yat+1)*w)]<lowestNeighbour&&numArray[xat-1+((yat+1)*w)]>0&&numArray[xat+((yat+1)*w)]>0&&numArray[xat-1+((yat)*w)]>0)
						{
							lowestNeighbour=numArray[xat-1+((yat+1)*w)];
							dir="LEFTDOWN";
						}
					}
					if(dir == "LEFT") //Find the direction to do the next iteration with
					{
						xat=xat-1;
					}
					else if(dir == "RIGHT")
					{
						xat=xat+1;
					}
					else if(dir == "UP")
					{
						yat=yat-1;
					}
					else if(dir == "DOWN")
					{
						yat=yat+1;
					}
					else if(dir == "LEFTUP")
					{
						xat=xat-1;
						yat=yat-1;
					}
					else if(dir == "RIGHTUP")
					{
						xat=xat+1;
						yat=yat-1;
					}
					else if(dir == "LEFTDOWN")
					{
						xat=xat-1;
						yat=yat+1;
					}
					else if(dir == "RIGHTDOWN")
					{
						xat=xat+1;
						yat=yat+1;
					}
					else
					{
						//System.out.println("ERROR: "+dir);
					}
				}
			}
			return int2s;
		}
	}
}