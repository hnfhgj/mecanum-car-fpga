// Standalone synthesizable BFS over a caller-supplied, footprint-inflated,
// observed-free grid. CELL_MM=50, 64x64 cells cover 3.2m. Not yet wired to car.
// Write ALL cells before requesting start; map writes during busy are rejected.
// Do not connect raw endpoint occupancy directly: unknown/inflated cells=0.
module square_grid_route #(
 parameter integer SIDE_BITS=6,
 parameter integer INDEX_BITS=2*SIDE_BITS,
 parameter integer CELLS=1<<INDEX_BITS,
 parameter integer EXPLORE=0
)(input wire clk,rst_n,cancel,
 input wire map_we,input wire [INDEX_BITS-1:0] map_addr,input wire map_free,
 input wire start,input wire [INDEX_BITS-1:0] origin,goal,
 output reg busy,done,found,output reg [1:0] direction,
 output reg write_rejected,
 input wire new_goal,output reg goal_reached);
 reg free_map[0:CELLS-1];
 reg seen[0:CELLS-1];
 reg free_src,free_dst,free_neighbour,seen_neighbour,visited_current;
 reg [INDEX_BITS-1:0] queue[0:CELLS-1];
 reg [1:0] first_step[0:CELLS-1];
 reg queue_we;reg [INDEX_BITS-1:0] queue_wa,queue_ra,queue_wdata,queue_rdata;
 reg [1:0] first_wdata,first_rdata;
 reg visited[0:CELLS-1];reg reset_visits,best_valid;
 reg [INDEX_BITS:0] best_cost;reg [1:0] best_direction;
 reg [INDEX_BITS:0] head,tail,clear_index;
 reg [INDEX_BITS-1:0] src,dst,current,neighbour;
 reg [1:0] current_first,edge_index;
 reg edge_ok;reg [3:0] state;
 wire [SIDE_BITS-1:0] cx=current[SIDE_BITS-1:0];
 wire [SIDE_BITS-1:0] cy=current[INDEX_BITS-1:SIDE_BITS];
 function [INDEX_BITS:0] taxi(input [INDEX_BITS-1:0] p,q);
 integer ax,ay,bx,by;
 begin ax=p% (1<<SIDE_BITS);ay=p>>SIDE_BITS;bx=q%(1<<SIDE_BITS);by=q>>SIDE_BITS;
  taxi=(ax>bx?ax-bx:bx-ax)+(ay>by?ay-by:by-ay);end endfunction
 always @*begin
  neighbour=current;edge_ok=0;
  case(edge_index)
   0:begin neighbour=current+1'b1;edge_ok=cx!={SIDE_BITS{1'b1}};end // +X
   1:begin neighbour=current+(1<<SIDE_BITS);edge_ok=cy!={SIDE_BITS{1'b1}};end // +Y
   2:begin neighbour=current-1'b1;edge_ok=cx!=0;end // -X
   3:begin neighbour=current-(1<<SIDE_BITS);edge_ok=cy!=0;end // -Y
  endcase
 end
 always @(posedge clk)begin
  if(map_we && !busy)free_map[map_addr]<=map_free;
  free_src<=free_map[src];free_dst<=free_map[dst];free_neighbour<=free_map[neighbour];
  seen_neighbour<=seen[neighbour];visited_current<=visited[current];
  if(state==1)seen[clear_index[INDEX_BITS-1:0]]<=0;
  else if(state==2 && free_src && (EXPLORE || free_dst) && src!=dst)seen[src]<=1;
  else if(state==11 && edge_ok && free_neighbour && !seen_neighbour)seen[neighbour]<=1;
  if(state==1)begin
   if(reset_visits)visited[clear_index[INDEX_BITS-1:0]]<=0;
   else if(taxi(clear_index[INDEX_BITS-1:0],src)<3)visited[clear_index[INDEX_BITS-1:0]]<=1;
  end
  if(queue_we)begin queue[queue_wa]<=queue_wdata;first_step[queue_wa]<=first_wdata;end
  queue_rdata<=queue[queue_ra];first_rdata<=first_step[queue_ra];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;found<=0;direction<=0;state<=0;
   head<=0;tail<=0;clear_index<=0;src<=0;dst<=0;current<=0;
   current_first<=0;edge_index<=0;write_rejected<=0;goal_reached<=0;
   reset_visits<=1;best_valid<=0;best_cost<=0;best_direction<=0;
   queue_we<=0;queue_wa<=0;queue_ra<=0;queue_wdata<=0;first_wdata<=0;
  end else begin
   done<=0;queue_we<=0;write_rejected<=map_we && busy;
   if(new_goal)reset_visits<=1;
   if(cancel)begin busy<=0;found<=0;goal_reached<=0;state<=0;end
   else case(state)
    0:if(start)begin src<=origin;dst<=goal;busy<=1;found<=0;goal_reached<=0;
     best_valid<=0;best_cost<={INDEX_BITS+1{1'b1}};clear_index<=0;state<=1;end
    1:begin
     if(clear_index==CELLS-1)begin state<=2;reset_visits<=0;end else clear_index<=clear_index+1'b1;
    end
    2:begin
     if(!free_src || (!EXPLORE && !free_dst))begin busy<=0;done<=1;state<=0;end
     else if(src==dst)begin busy<=0;done<=1;found<=1;goal_reached<=1;direction<=0;state<=0;end
     else begin queue_we<=1;queue_wa<=0;queue_wdata<=src;first_wdata<=0;
      head<=0;tail<=1;state<=3;end
    end
    3:begin
     if(head==tail)begin busy<=0;done<=1;state<=0;
      if(EXPLORE && best_valid)begin found<=1;direction<=best_direction;end
     end
     else begin queue_ra<=head[INDEX_BITS-1:0];state<=8;end
    end
    8:state<=9;
    9:begin current<=queue_rdata;current_first<=first_rdata;
     head<=head+1'b1;edge_index<=0;state<=4;end
    4:state<=10;
    10:state<=11;
    11:begin
     if(EXPLORE && edge_index==0 && !visited_current && taxi(current,src)>=3 && taxi(current,dst)<best_cost)begin
      best_valid<=1;best_cost<=taxi(current,dst);best_direction<=current_first;
     end
     if(edge_ok && free_neighbour && !seen_neighbour)begin
      if(neighbour==dst)begin
       direction<=current==src?edge_index:current_first;found<=1;goal_reached<=1;done<=1;busy<=0;state<=0;
      end else if(tail<CELLS)begin
       queue_we<=1;queue_wa<=tail[INDEX_BITS-1:0];queue_wdata<=neighbour;
       first_wdata<=current==src?edge_index:current_first;
       tail<=tail+1'b1;
       if(edge_index==3)state<=3;else begin edge_index<=edge_index+1'b1;state<=4;end
      end else begin busy<=0;done<=1;found<=0;state<=0;end
     end else if(edge_index==3)state<=3;else begin edge_index<=edge_index+1'b1;state<=4;end
    end
    default:begin state<=0;busy<=0;found<=0;end
   endcase
  end
 end
endmodule
