function [p0,q0,fep,feq,feq0,n1,d1,n2,d2] = keyPointOfISS(P,Q, r, e1,e2,idx,dis)
% idx_feature = My_ISS(p, r, e1,e2,idx,dis)  : ISS����ǩ����������ȡ
%
% https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
% �������
%    p    : ���ƾ���  3 * n
%    r    : ����뾶  ��λ��m
%    e1   : �м�����ֵ���������ֵ֮�ȵ� ��ֵ
%    e2   : �м�����ֵ����С����ֵ֮�ȵ� ��ֵ
%    idx  : �����������Ԫ�����飬��С����������򣬵�һ�����������ĵ㣬����ѯ��
%    dis  : �������룬Ԫ�����飬��С����������򣬵�һ�����������ĵ㣬����ѯ��
%
% �������
%   idx_feature  : ����p��ISS����������  pΪ3*n  idx_featureΪ p ������
%
%
%
%
%  Author��https://download.csdn.net/download/xxiaotouming/10325431?utm_medium=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-2~default~OPENSEARCH~Rate-4-10325431-download-16120741.topnsimilar_compare_v2&dest=https%3A%2F%2Fdownload.csdn.net%2Fdownload%2Fxxiaotouming%2F10325431&spm=1003.2020.3001.6616.4
%          adiusted by GJT
%  E-mail��gjt0114@outlook.com  of GJT



  %��ʾ
  displayer = displayFunction;                %for Debug

  global r_P_k r_Q_k;

  %400�������
  [n1,d1,n2,d2]=Knncaculate(P,Q);

%���׸��� 2.1.1����Ӧ����ƽ���������
  paper = paperISS;
  r_P_k = paper.paper(d1) * 6;
  r_Q_k = paper.paper(d2) * 6;
 

% for i=1:15                                %for Debug

  %ISS�ؼ�����ȡ
    % [p0,q0,fep,feq,feq0]=ISSCaculate(P ,Q ,r ,r ,e1 ,e2 ,idx ,dis);
    [p0,q0,fep,feq,feq0]=ISSCaculate(P ,Q ,r_P_k  ,r_Q_k,e1 ,e2 );

    % displayer.displayFinalPickKeyPoint(p0,q0); %for Debug
    figure;
    % plot3(P(1,:),P(2,:),P(3,:),'y.');
    % hold on
    plot3(p0(1,:),p0(2,:),p0(3,:),'r.');
    title(num2str(size(p0,2)));
    % play_ball(p0);
    % figure;
    % plot3(q0(1,:),q0(2,:),q0(3,:),'b.');

%���׸��� 2.2����������ı߽���ж� �Ż��ؼ���
    % border_point = paper.borderPoint(P);
    p0 = paper.borderPoint(P,p0);
    

    % num_p0_r(i) = size(p0,2);                %for Debug

%     figure;                                      %for Debug
    % displayer.displayFinalPickKeyPoint(p0,q0); %for Debug
    hold on
    plot3(p0(1,:),p0(2,:),p0(3,:),'b.');
    title(num2str(size(p0,2)),['e1=',num2str(e1),'e2=',num2str(e2)]);
    % play_ball(p0);
     axis equal;
    % i                                        %for Debug
% end                                        %for Debug

  % figure                                     %for Debug
  % plot(num_p0_r)                             %for Debug 
  % xlabel('i * den')                          %for Debug


end


function [n1,d1,n2,d2]=Knncaculate(P,Q)

  [n1,d1] = knn(P);
  [n2,d2] = knn(Q);

end

function [n1,d1] = knn(P)
    [n1,d1] = knnsearch(transpose(P), transpose(P), 'k', 400);      %����ȡ���� �����400���� ��
                                                                    %n1Ϊ���صĵ�����������վ����������
                                                                    %d1Ϊ������õ�ľ��룬��������
                                                                    %���Ϊn*400��
    n1=transpose(n1);                       %�ֱ��n1��d1ȡת��Ϊ400*n��
    d1=transpose(d1);

end

function [p0,q0,fep,feq,feq0]=ISSCaculate(P,Q, r_P,r_Q, e1,e2)
% ����ISS�ؼ���
    fep = My_ISS(P, r_P, e1,e2);
    feq = My_ISS(Q, r_Q, e1,e2);

    feq0 = feq;

    p0 = P(:,fep);
    q0 = Q(:,feq);


end



function play_ball(p0)
    global r_P_k r_Q_k;
    % ���Ƽ�����
    aplha=0:pi/40:2*pi;
    for i=1:10:size(p0,2)

        % r=2;        
        x=r_P_k *cos(aplha) + p0(1,i);
        y=r_P_k *sin(aplha) + p0(2,i);
        hold on
        % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
        plot3(x,y,repelem(p0(3,i),size(x,2)),'g-')
        plot3(p0(1,i),p0(2,i),p0(3,i),'g*');

        x=r_P_k *cos(aplha) + p0(1,i);
        z=r_P_k *sin(aplha) + p0(3,i);
        hold on
        % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
        plot3(x,repelem(p0(2,i),size(x,2)),z,'g-')

        y=r_P_k *cos(aplha) + p0(2,i);
        z=r_P_k *sin(aplha) + p0(3,i);
        hold on
        % plot3(x,y,repmat(int32(p0(3,1)),size(x),1),'b-');
        plot3(repelem(p0(1,i),size(x,2)),y,z,'g-')
        hold off
        
    end

end
