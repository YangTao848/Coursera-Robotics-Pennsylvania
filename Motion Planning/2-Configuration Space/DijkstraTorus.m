function route = DijkstraTorus (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);


[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

[Y, X] = meshgrid (1:ncols, 1:nrows);
xd = dest_coords(1);
yd = dest_coords(2);


% Evaluate Heuristic function, H, for each grid cell
% Manhattan distance
H = (X - xd).^2 + (Y - yd).^2;

% Initialize cost arrays
f = Inf(nrows,ncols);
g = Inf(nrows,ncols);

g(start_node) = 0;
f(start_node) = H(start_node);

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    % Find the node with the minimum distance
    [min_dist, current] = min(f(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    f(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(f), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
    
        
    if (i+1>nrows)
        neighbor=sub2ind(size(map),1,j);
    else
        neighbor=sub2ind(size(map),i+1,j);
    end
    m=map(neighbor);
    if(map(neighbor)~=2 && map(neighbor)~=3 && map(neighbor)~=5)
        map(neighbor)=4;
       if(g(neighbor)>(g(current)+1))
                g(neighbor)=g(current)+1;
                f(neighbor)=g(neighbor)+H(neighbor);
                parent(neighbor)=current;
            end
    end
    
    
    if (i-1<1)
        neighbor=sub2ind(size(map),nrows,j);
    else
        neighbor=sub2ind(size(map),i-1,j);
    end
        map(neighbor);
        if(map(neighbor)~=2 && map(neighbor)~=3 && map(neighbor)~=5)
            map(neighbor)=4;
           if(g(neighbor)>(g(current)+1))
                g(neighbor)=g(current)+1;
                f(neighbor)=g(neighbor)+H(neighbor);
                parent(neighbor)=current;
            end
        end
   
    
    if (j+1>ncols)
        neighbor=sub2ind(size(map),i,1);
    else
        neighbor=sub2ind(size(map),i,j+1);
    end
        map(neighbor);
        if(map(neighbor)~=2 && map(neighbor)~=3 && map(neighbor)~=5)
            map(neighbor)=4;
            if(g(neighbor)>(g(current)+1))
                g(neighbor)=g(current)+1;
                f(neighbor)=g(neighbor)+H(neighbor);
                parent(neighbor)=current;
            end
        end
 
    
    if (j-1<1)
        neighbor=sub2ind(size(map),i,ncols);
    else
        neighbor=sub2ind(size(map),i,j-1);
    end
        map(neighbor);
        if(map(neighbor)~=2 && map(neighbor)~=3 && map(neighbor)~=5)
            map(neighbor)=4;
            if(g(neighbor)>(g(current)+1))
                g(neighbor)=g(current)+1;
                f(neighbor)=g(neighbor)+H(neighbor);
                parent(neighbor)=current;
            end
        end
 

    
    % *******************************************************************
end

if (isinf(f(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
end

    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (f(i,j) > d) )
            f(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

end
