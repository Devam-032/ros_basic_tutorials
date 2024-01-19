#!/usr/bin/env python3

def traversal_logic(dir_index ,diro_index, dir,dir_count,count,y,leny,Goal):  #dir_index shold be under 0 and leny left_index and right_index can less than 0 or more leny resp.
                    index_temp = dir_index 

                    while(count<7):

                        if(y[index_temp]<0.2 ):
                            count = count+1
                            dir_index = dir_index +(2*dir-1)
                            dir_count = dir_count + 1
                            index_temp = dir_index
                            if(dir_index==dir*leny + (dir-1)):  #if(left_index == -1 or right_index == leny)
                              dir_index = (-dir+1)*leny + (dir-1) #left_index  = leny-1 right_index= 0
                              index_temp = dir_index
                            if( dir_index==diro_index or dir_index == Goal):
                              if(count==7):
                                   return dir_index,dir_count,count
                              else:
                                   count = 0
                                   return -1,dir_count,count
                            
                        else:
                            count = 0
                            dir_index = dir_index +(2*dir-1)
                            dir_count = dir_count + 1
                            index_temp = dir_index
                            if(dir_index == dir*leny + (dir-1)):
                                 dir_index = (-dir+1)*leny + (dir-1)
                                 index_temp = dir_index
                            return dir_index,dir_count,count

                    return dir_index,dir_count,count
                    


def traversal(count,left_count,right_count,left_index,right_index,y,leny,Goal):
            #while(count<9 or (abs(int(leny/2)-left_index)<int(leny/2)-7 and abs(int(leny/2)-right_index)<int(leny/2)-7)):
            while(count<7):
                if(Goal<abs(int(len(y)/2))):
                    # if(left_count<=right_count ):
                         count = 0
                         dir = 0
                         left_index ,left_count ,count = traversal_logic(left_index ,right_index, dir,left_count,count,y,leny,Goal)
                         if(left_index==-1):
                              return -1
                         
                else:    
                    # elif(left_count>right_count ):
                         count = 0
                         dir = 1
                         right_index ,right_count ,count = traversal_logic(right_index ,left_index, dir,right_count,count,y,leny,Goal)
                         if(right_index == -1):
                              return -1
                         
               

            if(dir==0 and count==7):
                if(left_index>=leny-3):
                     return left_index-leny+3
                else:
                    return left_index+3

            elif(dir==1 and count==7):
                if(right_index<=3):
                     return leny-3+right_index
                else:
                    return right_index-3
            else:
                 return -1
        

y = [0.0, 0.0, 0.0, 0.051994902747018, 0.15213957514081683, 0.20451817512512208, 0.24405434131622314, 0.2755889960697719, 0.29913625717163084, 0.3083496400288173, 0.29081928389413014, 0.2636041402816772, 0.22666607243674142, 0.03999864033290318, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.036513580594744, 0.20837278366088868, 0.24481570039476666, 0.2693335941859654, 0.28386380672454836, 0.26499131407056536, 0.23463453565325057, 0.19227113042558944, 0.12682649067470006, 0.029284402302333288, 0.0, 0.0, 0.0]

leny = len(y)
print(leny)
count ,sumn , sumb = 0 , 0 , 0
Goal =0
index_temp = Goal
left_index = index_temp
right_index = index_temp
left_count = index_temp - left_index #initially zero
right_count = right_index - index_temp #initially zero
a =4
for i in range (1,a):
        index_temp1 , index_temp2 = Goal,Goal
        if (index_temp1 -i) < 0:
            index_temp1 = leny + (index_temp1 - i) + i  # if index_temp - i = -1 then index_temp -i = leny + (index_temp -i)
        elif (index_temp2 +i) > leny-1:
            index_temp2 = (index_temp2+i) - leny -i
        sumn = sumn + y[index_temp1-i]
        sumb = sumb + y[index_temp2+i]
print(sumn,sumb)
if(y[Goal]==0 and sumn <(a-1)*0.2 and sumb <(a-1)*0.2):

        index_final = Goal
        sumn = 0 
        sumb = 0

else:
        index_final = traversal(count,left_count,right_count,left_index,right_index,y,leny,Goal)
        sumn = 0
        sumb = 0


print(index_final)