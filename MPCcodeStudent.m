%Function for the MPC: 
function u=MPCcodeStudent(ref,P0,Ts,H)
%ref is a reference vector.
%P0 is the current position state of the robot. P0=[x;y].
%Ts is the sample time of the MPC.
%H is the prediction Horizon.
%======================================================
%Define the weights for the MPC: 
%Error tracking weights: 
Q = [1 0; 0 1];
%Input weights:
R = [1 0; 0 1];
%limits on the velocities:
vlim=0.22;
H1 = [1 1];
%Reshape the reference into a column vector for the lifted states:
ref=reshape(ref,[2*size(ref,2) 1]);
%Define the model matrices: 
%State Matrix:
A = [1 0; 0 1];
%Input Matrix:
B = [Ts 0; 0 Ts];
%matrix for the constraints: 
ylim=-0.6;
G1=[0 -1]; %To extract the y position

%Lifting (Al "= [A^* A^*B]" the "full" lifted matrix):
Al=zeros(2*H,(H+1)*2);
Al(1:2,1:2)=A;
Al(1:2,(1+2):(2+2))=B;
for j=1:(H-1)
    Al((1+2*j):(2+2*j),:)=A*Al((1+2*(j-1)):(2+2*(j-1)),:);
    Al((1+2*j):(2+2*j),(1+2*(j+1)):(2+2*(j+1)))=B;
end

Qcell=repmat({Q},1,H);
Rcell=repmat({R},1,H);
G1cell=repmat({G1},1,H);
H1cell=repmat({H1},1,H);
Ql=blkdiag(Qcell{:});
Rl=blkdiag(Rcell{:});
G1l=blkdiag(G1cell{:});
H1l=blkdiag(H1cell{:});

cvx_begin quiet
    variable u(2*H,1);
    z = Al * vertcat(P0,u);
    minimize((z-ref)'*Ql*(z-ref) + u'*Rl*u);
subject to 
    g = repelem([ylim],H)';
    horzcat(G1l,g) * vertcat(z,1) <= 0;
    
    h = repelem([-vlim],H)';
    horzcat(H1l,h) * vertcat(z,1) <= 0;
    
cvx_end 
end
