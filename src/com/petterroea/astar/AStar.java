package com.petterroea.astar;

import java.awt.Point;
import java.util.LinkedList;
import java.util.List;

/**
 * ASTAR - Lightweight A* pathfinding algoritm implementation in Java
 * Copyright (C) 2012  Liam Svanåsbakken Crouch(Known as Liam S. Crouch internationally) aka petterroea
 * 
 * You should of gotten a copy of the Licence with this source code, please see "LICENCE"
 * 
 * @author Petterroea
 *
 */
public class AStar {
	/**
	 * Gives you a path from start to end based on a map, where only straight paths are allowed(No diagnoal)
	 * 
	 * Please note that it only takes a 1-dimensional array, because that is more proffesional. Google converting a 2dimensional array to a 1dimensional array in java ;).
	 * @param map An array of booleans. False is air, true is wall
	 * @param w Width of map
	 * @param h Height of map
	 * @param start The start point
	 * @param end The end point
	 * @return A list containing all points in the best path. If no path is found, the list will be empty.
	 * @throws IllegalArgumentException if one of the points are outside the area
	 */
	public static List<Point> getPath(boolean[] map, int w, int h, Point start, Point end)
	{
		//First, some checking
		if(start.x>=w||start.y>=h||start.x<0||start.y<0)
		{
			throw new java.lang.IllegalArgumentException("Start is outside of array!");
		}
		if(end.x>=w||end.y>=h||end.x<0||end.y<0)
		{
			throw new java.lang.IllegalArgumentException("End is outside of array!");
		}
		List<Point> todoList = new LinkedList<Point>();//List of points to do for the current iteration
		todoList.add(start);//Add a start point.
		int[] numArray = new int[map.length]; //Array containing length weight
		boolean foundExit=false; //So we can detect if we have reached the end
		numArray[start.x+(start.y*w)]=1; //Set start weight number
		while(todoList.size()>0&&foundExit==false) //While we still have points to scan, and we have not reached the exit
		{
			List<Point> nextIteration = new LinkedList<Point>(); //List of points to scan for next iteration
			for(int i = 0; i < todoList.size(); i++)//Iterate over the list
			{
				int x = todoList.get(i).x;
				int y = todoList.get(i).y;
				if(x==end.x&&y==end.y)
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
						nextIteration.add(new Point(x+1, y));
						numArray[x+1+(y*w)]=thisNum+1;
					}
				}
				if(x>0)
				{
					if(!map[x-1+(y*w)]&&(numArray[x-1+(y*w)]>thisNum+1||numArray[x-1+(y*w)]==0))
					{
						nextIteration.add(new Point(x-1, y));
						numArray[x-1+(y*w)]=thisNum+1;
					}
				}
				if(y<h-1)
				{
					if(!map[x+((y+1)*w)]&&(numArray[x+((y+1)*w)]>thisNum+1||numArray[x+((y+1)*w)]==0))
					{
						nextIteration.add(new Point(x, y+1));
						numArray[x+((y+1)*w)]=thisNum+1;
					}
				}
				if(y>0)
				{
					if(!map[x+((y-1)*w)]&&(numArray[x+((y-1)*w)]>thisNum+1||numArray[x+((y-1)*w)]==0))
					{
						nextIteration.add(new Point(x, y-1));
						numArray[x+((y-1)*w)]=thisNum+1;
					}
				}
			}
			todoList=nextIteration; //Set the todo list as the new iteration-list
		}
//		for(int y = 0; y < h; y++)
//		{
//			String buffer="";
//			for(int x = 0; x < w; x++)
//			{
//				if(x==start.x&&y==start.y)
//				{
//					buffer=buffer+"S  ";
//				}
//				else if(x==end.x&&y==end.y)
//				{
//					buffer=buffer+"E  ";
//				}
//				else
//				{
//					int num = numArray[x+(y*w)];
//					if(num>9)
//					{
//						buffer=buffer+num+" ";
//					}
//					else
//					{
//						buffer=buffer+num+"  ";
//					}
//				}
//			}
//			System.out.println(buffer);
//		}
		List<Point> points = new LinkedList<Point>(); //We are done scanning, so we init a list of all points in the path back.
		if(foundExit)
		{
			boolean gotBack = false;
			int xat=end.x;
			int yat=end.y;
			while(!gotBack)
			{
				points.add(new Point(xat, yat));
 				if(xat==start.x&&yat==start.y) //We found the way back
				{
					gotBack=true;
					//System.out.println("Got back");
					break;
				}
				int arrayPos=xat+(yat*w); //Get current position and store it in a variable, to make the code more readable.
				int thisNum=numArray[arrayPos]; //Get current weight and store it in a variable, to make the code more readable.
				int lowestNeighbour=thisNum; //Store lowest neighbour weight, so we can decide what neighbour is closest to start.
				String dir="NONE"; //Store the direction in a String, because i can.
				if(xat>0) //Only check left side if there is a spot to the left
				{
					if(numArray[xat-1+(yat*w)]<lowestNeighbour&&numArray[xat-1+(yat*w)]>0)
					{
						lowestNeighbour=numArray[xat-1+(yat*w)];
						dir="LEFT";
						//System.out.println("YES: " + thisNum + ", LEFT: " + numArray[xat-1+(yat*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", LEFT: " + numArray[xat-1+(yat*w)]);
//					}
				}
				if(yat>0) //Only check up if there is a spot upwards
				{
					if(numArray[xat+((yat-1)*w)]<lowestNeighbour&&numArray[xat+((yat-1)*w)]>0)
					{
						lowestNeighbour=numArray[xat+((yat-1)*w)];
						dir="UP";
						//System.out.println("YES: " + thisNum + ", UP: " + numArray[xat+((yat-1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", UP: " + numArray[xat+((yat-1)*w)]);
//					}
				}
				if(xat<w-1) //Only check right if there is a spot to the right
				{
					if(numArray[xat+1+(yat*w)]<lowestNeighbour&&numArray[xat+1+(yat*w)]>0)
					{
						lowestNeighbour=numArray[xat+1+(yat*w)];
						dir="RIGHT";
						//System.out.println("YES: " + thisNum + ", RIGHT: " + numArray[xat+1+(yat*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", RIGHT: " + numArray[xat+1+(yat*w)]);
//					}
				}
				if(yat<h-1) //Only check down if there is a spot downwards.
				{
					if(numArray[xat+((yat+1)*w)]<lowestNeighbour&&numArray[xat+((yat+1)*w)]>0)
					{
						lowestNeighbour=numArray[xat+((yat+1)*w)];
						dir="DOWN";
						//System.out.println("YES: " + thisNum + ", DOWN: " + numArray[xat+((yat+1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", DOWN: " + numArray[xat+((yat+1)*w)]);
//					}
				}
				if(dir.equals("LEFT")) //Find the direction to do the next iteration with
				{
					xat=xat-1;
				}
				else if(dir.equals("RIGHT"))
				{
					xat=xat+1;
				}
				else if(dir.equals("UP"))
				{
					yat=yat-1;
				}
				else if(dir.equals("DOWN"))
				{
					yat=yat+1;
				}
				else
				{
					System.out.println("ERROR: "+dir);
				}
			}
		}
		return points;
	}
	/**
	 * Gives you a path from start to end based on a map, where diagnoal paths are allowed.
	 * 
	 * Please note that it only takes a 1-dimensional array, because that is more proffesional. Google converting a 2dimensional array to a 1dimensional array in java ;).
	 * @param map An array of booleans. False is air, true is wall
	 * @param w Width of map
	 * @param h Height of map
	 * @param start The start point
	 * @param end The end point
	 * @return A list containing all points in the best path. If no path is found, the list will be empty.
	 * @throws IllegalArgumentException if one of the points are outside the area
	 */
	public static List<Point> getPathWithDiagnoal(boolean[] map, int w, int h, Point start, Point end)
	{
		//First, some checking
		if(start.x>=w||start.y>=h||start.x<0||start.y<0)
		{
			throw new java.lang.IllegalArgumentException("Start is outside of array!");
		}
		if(end.x>=w||end.y>=h||end.x<0||end.y<0)
		{
			throw new java.lang.IllegalArgumentException("End is outside of array!");
		}
		List<Point> todoList = new LinkedList<Point>();//List of points to do for the current iteration
		todoList.add(start);//Add a start point.
		int[] numArray = new int[map.length]; //Array containing length weight
		boolean foundExit=false; //So we can detect if we have reached the end
		numArray[start.x+(start.y*w)]=1; //Set start weight number
		while(todoList.size()>0&&foundExit==false) //While we still have points to scan, and we have not reached the exit
		{
			List<Point> nextIteration = new LinkedList<Point>(); //List of points to scan for next iteration
			for(int i = 0; i < todoList.size(); i++)//Iterate over the list
			{
				int x = todoList.get(i).x;
				int y = todoList.get(i).y;
				if(x==end.x&&y==end.y)
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
						nextIteration.add(new Point(x+1, y));
						numArray[x+1+(y*w)]=thisNum+1;
					}
				}
				if(x>0)
				{
					if(!map[x-1+(y*w)]&&(numArray[x-1+(y*w)]>thisNum+1||numArray[x-1+(y*w)]==0))
					{
						nextIteration.add(new Point(x-1, y));
						numArray[x-1+(y*w)]=thisNum+1;
					}
				}
				if(y<h-1)
				{
					if(!map[x+((y+1)*w)]&&(numArray[x+((y+1)*w)]>thisNum+1||numArray[x+((y+1)*w)]==0))
					{
						nextIteration.add(new Point(x, y+1));
						numArray[x+((y+1)*w)]=thisNum+1;
					}
				}
				if(y>0)
				{
					if(!map[x+((y-1)*w)]&&(numArray[x+((y-1)*w)]>thisNum+1||numArray[x+((y-1)*w)]==0))
					{
						nextIteration.add(new Point(x, y-1));
						numArray[x+((y-1)*w)]=thisNum+1;
					}
				}
			}
			todoList=nextIteration; //Set the todo list as the new iteration-list
		}
//		for(int y = 0; y < h; y++)
//		{
//			String buffer="";
//			for(int x = 0; x < w; x++)
//			{
//				if(x==start.x&&y==start.y)
//				{
//					buffer=buffer+"S  ";
//				}
//				else if(x==end.x&&y==end.y)
//				{
//					buffer=buffer+"E  ";
//				}
//				else
//				{
//					int num = numArray[x+(y*w)];
//					if(num>9)
//					{
//						buffer=buffer+num+" ";
//					}
//					else
//					{
//						buffer=buffer+num+"  ";
//					}
//				}
//			}
//			System.out.println(buffer);
//		}
		List<Point> points = new LinkedList<Point>(); //We are done scanning, so we init a list of all points in the path back.
		if(foundExit)
		{
			boolean gotBack = false;
			int xat=end.x;
			int yat=end.y;
			while(!gotBack)
			{
				points.add(new Point(xat, yat));
 				if(xat==start.x&&yat==start.y) //We found the way back
				{
					gotBack=true;
					//System.out.println("Got back");
					break;
				}
				int arrayPos=xat+(yat*w); //Get current position and store it in a variable, to make the code more readable.
				int thisNum=numArray[arrayPos]; //Get current weight and store it in a variable, to make the code more readable.
				int lowestNeighbour=thisNum; //Store lowest neighbour weight, so we can decide what neighbour is closest to start.
				String dir="NONE"; //Store the direction in a String, because i can.
				if(xat>0) //Only check left side if there is a spot to the left
				{
					if(numArray[xat-1+(yat*w)]<lowestNeighbour&&numArray[xat-1+(yat*w)]>0)
					{
						lowestNeighbour=numArray[xat-1+(yat*w)];
						dir="LEFT";
						//System.out.println("YES: " + thisNum + ", LEFT: " + numArray[xat-1+(yat*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", LEFT: " + numArray[xat-1+(yat*w)]);
//					}
				}
				if(yat>0) //Only check up if there is a spot upwards
				{
					if(numArray[xat+((yat-1)*w)]<lowestNeighbour&&numArray[xat+((yat-1)*w)]>0)
					{
						lowestNeighbour=numArray[xat+((yat-1)*w)];
						dir="UP";
						//System.out.println("YES: " + thisNum + ", UP: " + numArray[xat+((yat-1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", UP: " + numArray[xat+((yat-1)*w)]);
//					}
				}
				if(xat<w-1) //Only check right if there is a spot to the right
				{
					if(numArray[xat+1+(yat*w)]<lowestNeighbour&&numArray[xat+1+(yat*w)]>0)
					{
						lowestNeighbour=numArray[xat+1+(yat*w)];
						dir="RIGHT";
						//System.out.println("YES: " + thisNum + ", RIGHT: " + numArray[xat+1+(yat*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", RIGHT: " + numArray[xat+1+(yat*w)]);
//					}
				}
				if(yat<h-1) //Only check down if there is a spot downwards.
				{
					if(numArray[xat+((yat+1)*w)]<lowestNeighbour&&numArray[xat+((yat+1)*w)]>0)
					{
						lowestNeighbour=numArray[xat+((yat+1)*w)];
						dir="DOWN";
						//System.out.println("YES: " + thisNum + ", DOWN: " + numArray[xat+((yat+1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", DOWN: " + numArray[xat+((yat+1)*w)]);
//					}
				}
				if(yat>0&&xat>0)
				{
					if(numArray[xat-1+((yat-1)*w)]<lowestNeighbour&&numArray[xat-1+((yat-1)*w)]>0&&numArray[xat+((yat-1)*w)]>0&&numArray[xat-1+((yat)*w)]>0)
					{
						lowestNeighbour=numArray[xat-1+((yat-1)*w)];
						dir="LEFTUP";
						//System.out.println("YES: " + thisNum + ", LEFTUP: " + numArray[xat-1+((yat-1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", LEFTUP: " + numArray[xat-1+((yat-1)*w)]);
//					}
				}
				if(yat>0&&xat<h-1)
				{
					if(numArray[xat+1+((yat-1)*w)]<lowestNeighbour&&numArray[xat+1+((yat-1)*w)]>0&&numArray[xat+((yat-1)*w)]>0&&numArray[xat+1+((yat)*w)]>0)
					{
						lowestNeighbour=numArray[xat+1+((yat-1)*w)];
						dir="RIGHTUP";
						//System.out.println("YES: " + thisNum + ", RIGHTUP: " + numArray[xat+1+((yat-1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", RIGHTUP: " + numArray[xat+1+((yat-1)*w)]);
//					}
				}
				if(yat<w-1&&xat<h-1)
				{
					if(numArray[xat+1+((yat+1)*w)]<lowestNeighbour&&numArray[xat+1+((yat+1)*w)]>0&&numArray[xat+((yat+1)*w)]>0&&numArray[xat+1+((yat)*w)]>0)
					{
						lowestNeighbour=numArray[xat+1+((yat+1)*w)];
						dir="RIGHTDOWN";
						//System.out.println("YES: " + thisNum + ", RIGHTDOWN: " + numArray[xat+1+((yat+1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", RIGHTDOWN: " + numArray[xat+1+((yat+1)*w)]);
//					}
				}
				if(yat<w-1&&xat>0)
				{
					if(numArray[xat-1+((yat+1)*w)]<lowestNeighbour&&numArray[xat-1+((yat+1)*w)]>0&&numArray[xat+((yat+1)*w)]>0&&numArray[xat-1+((yat)*w)]>0)
					{
						lowestNeighbour=numArray[xat-1+((yat+1)*w)];
						dir="LEFTDOWN";
						//System.out.println("YES: " + thisNum + ", LEFTDOWN: " + numArray[xat-1+((yat+1)*w)]);
					}
//					else
//					{
//						System.out.println("NO: " + thisNum + ", LEFTDOWN: " + numArray[xat-1+((yat+1)*w)]);
//					}
				}
				if(dir.equals("LEFT")) //Find the direction to do the next iteration with
				{
					xat=xat-1;
				}
				else if(dir.equals("RIGHT"))
				{
					xat=xat+1;
				}
				else if(dir.equals("UP"))
				{
					yat=yat-1;
				}
				else if(dir.equals("DOWN"))
				{
					yat=yat+1;
				}
				else if(dir.equals("LEFTUP"))
				{
					xat=xat-1;
					yat=yat-1;
				}
				else if(dir.equals("RIGHTUP"))
				{
					xat=xat+1;
					yat=yat-1;
				}
				else if(dir.equals("LEFTDOWN"))
				{
					xat=xat-1;
					yat=yat+1;
				}
				else if(dir.equals("RIGHTDOWN"))
				{
					xat=xat+1;
					yat=yat+1;
				}
				else
				{
					System.out.println("ERROR: "+dir);
				}
			}
		}
		return points;
	}
}
