function dataset = generateGATDatasetStage12(sampleCount,randomSeed)
%GENERATEGATDATASETSTAGE12 Synthetic fixed-graph threat dataset.

rng(randomSeed,'twister');
featureCount=19; nodeCount=17;
features=zeros(featureCount,nodeCount,sampleCount,'single');
adjacency=false(nodeCount,nodeCount,sampleCount);
mask=false(nodeCount,sampleCount);
labels=zeros(nodeCount,sampleCount,'single');
baseline=zeros(nodeCount,sampleCount,'single');

for sample=1:sampleCount
    activeTracks=randi([3,16]);
    mask(1,sample)=true;
    features(1,1,sample)=1;
    relativePosition=zeros(3,16);
    ownRisk=zeros(16,1);
    for track=1:activeTracks
        node=track+1;
        mask(node,sample)=true;
        type=randi(3);
        features(1+type,node,sample)=1;

        bearing=2*pi*rand;
        distance=0.35+2.65*rand;
        elevation=-0.35+0.70*rand;
        horizontal=sqrt(max(distance^2-elevation^2,0.01));
        r=[horizontal*cos(bearing);horizontal*sin(bearing);elevation];
        speed=0.05+1.15*rand;
        if rand<0.65
            w=-speed*r/max(norm(r),1e-6)+0.12*randn(3,1);
        else
            w=0.45*randn(3,1);
        end
        radius=0.08+0.32*rand;
        confidence=0.55+0.45*rand;
        trackAge=0.50*rand;
        if type==1
            source=1+(rand<0.55)*4; % 1 or 5
        elseif type==2
            source=2+(rand<0.55)*4; % 2 or 6
        else
            source=4;
        end
        centreDistance=norm(r);
        surfaceDistance=max(centreDistance-radius,0);
        closing=-dot(r,w)/max(centreDistance,1e-6);
        if closing>0.01
            ttc=surfaceDistance/closing;
        else
            ttc=1000;
        end

        relativePosition(:,track)=r;
        features(5:7,node,sample)=single(r/3);
        features(8:10,node,sample)=single(w/2);
        features(11,node,sample)=single(min(surfaceDistance/3,1));
        features(12,node,sample)=single(min(max(closing/2,-1),1));
        features(13,node,sample)=single(min(ttc/10,1));
        features(14,node,sample)=single(min(radius,1));
        features(15,node,sample)=single(confidence);
        features(16,node,sample)=single(trackAge/0.5);
        features(17,node,sample)=single(maskHas(source,1));
        features(18,node,sample)=single(maskHas(source,2));
        features(19,node,sample)=single(maskHas(source,4));

        % Future closest-approach expert label over a five-second horizon.
        speedSquared=dot(w,w);
        if speedSquared>1e-6
            timeClosest=min(max(-dot(r,w)/speedSquared,0),5);
        else
            timeClosest=5;
        end
        closestDistance=norm(r+w*timeClosest);
        safeDistance=0.22+radius+0.18+0.10*(1-confidence);
        clearanceClosest=closestDistance-safeDistance;
        geometricRisk=exp(-max(clearanceClosest,0)/0.45)*exp(-timeClosest/5);
        currentRisk=exp(-max(surfaceDistance-safeDistance,0)/0.65);
        closingRisk=min(max(closing/1.2,0),1);
        ownRisk(track)=min(1,confidence*(0.55*geometricRisk+ ...
            0.30*currentRisk+0.15*closingRisk));

        proximity=exp(-surfaceDistance/0.8);
        ttcTerm=(closing>0.01)*(1-min(ttc/8,1));
        closingTerm=min(max(closing,0),1);
        uncertainty=min((1-confidence)+trackAge/0.5,1);
        typeWeight=[0.85,0.75,1.0];
        baseline(node,sample)=single(min(1,typeWeight(type)*confidence*( ...
            0.45*proximity+0.30*ttcTerm+0.20*closingTerm)+0.05*uncertainty));
    end

    % Build the same radius graph as Stage 10.
    for node=2:activeTracks+1
        track=node-1;
        if norm(relativePosition(:,track))<=3
            adjacency(1,node,sample)=true; adjacency(node,1,sample)=true;
        end
    end
    for first=1:activeTracks
        for second=first+1:activeTracks
            if norm(relativePosition(:,first)-relativePosition(:,second))<=1.5
                adjacency(first+1,second+1,sample)=true;
                adjacency(second+1,first+1,sample)=true;
            end
        end
    end

    % Add modest relational/crowding risk from adjacent dangerous entities.
    for track=1:activeTracks
        node=track+1;
        neighborRisk=0;
        for other=1:activeTracks
            if other~=track && adjacency(node,other+1,sample)
                neighborRisk=max(neighborRisk,ownRisk(other));
            end
        end
        labels(node,sample)=single(min(1,ownRisk(track)+0.15*neighborRisk));
    end
end

trainingMask=mask;
trainingMask(1,:)=false; % ego is never a threat label

dataset.Features=features;
dataset.Adjacency=adjacency;
dataset.NodeMask=mask;
dataset.TrainingMask=trainingMask;
dataset.Labels=labels;
dataset.Baseline=baseline;
end

function present=maskHas(maskValue,bitValue)
present=mod(floor(maskValue/bitValue),2)==1;
end
