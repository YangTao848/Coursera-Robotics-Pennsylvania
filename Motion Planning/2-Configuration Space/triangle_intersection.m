function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

flag = true;
for i=1:3
   X=P1;
   X(i,:)=[];
   
   a=X(2,:)-X(1,:);
   if a(1)==0
       p1=P1(i,1)- X(1,1);
       p2=P2(:,1)- X(1,1);
   else
       a=a(2)/a(1);
       b=X(1,2)-a*X(1,1);

       p1=P1(i,2)- a*P1(i,1)-b;
       p2=P2(:,2)- a*P2(:,1)-b;   
     
   end
   
   p1=p1/abs(p1);
   p2=sum(p2./abs(p2));
   p2*p1;
   if p2*p1==-3
       flag=false;
       return
   end
end 


for i=1:3
   X=P2;
   X(i,:)=[];
   
   a=X(2,:)-X(1,:);
   if a(1)==0
       p2=P2(i,1)- X(1,1);
       p1=P1(:,1)- X(1,1);
   else
       a=a(2)/a(1);
       b=X(1,2)-a*X(1,1);

       p2=P2(i,2)- a*P2(i,1)-b;
       p1=P1(:,2)- a*P1(:,1)-b;   
     
   end
   
   p2=p2/abs(p2);
   p1=sum(p1./abs(p1));
   p2*p1;
   if p2*p1==-3
       flag=false;
       return
   end
end       

% *******************************************************************
end