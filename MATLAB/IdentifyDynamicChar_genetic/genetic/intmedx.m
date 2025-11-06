% intmedx - intermediate crossover
%
%	Description:
%	The function creates a new population of strings, which rises after
%	intermediate operation of all (couples of) strings of the old
%	population. The selection of strings into couples is random. alfa is
%	factor of appropriate genes crossover intensity.  
%
%
%	Syntax: 
%
%	Newpop=intmedx(Oldpop,alfa)
%
%	Newpop - new population 
%	Oldpop - old population
%	alfa - factor of crossover intensity of appropriate genes
%

% I.Sekaj, 10/2020

function[Newpop]=intmedx(Oldpop,alfa)

Newpop=Oldpop;
[lpop,lstring]=size(Oldpop);
flag=zeros(1,lpop);
num=fix(lpop/2);
i=1;

for cyk=1:num
    
    while flag(i)~=0
        i=i+1;
    end
    flag(i)=1;
    j=ceil(lpop*rand);
    while flag(j)~=0
        j=ceil(lpop*rand);
    end
    flag(j)=2;
    
    
    for g=1:lstring
        
        dx(g)=abs(Oldpop(i,g)-Oldpop(j,g));
        
        if Oldpop(i,g)<Oldpop(j,g)
            Newpop(i,g)=Oldpop(i,g)+rand*alfa*dx(g);
            Newpop(j,g)=Oldpop(j,g)-rand*alfa*dx(g);
        else
            Newpop(i,g)=Oldpop(i,g)-rand*alfa*dx(g);
            Newpop(j,g)=Oldpop(j,g)+rand*alfa*dx(g);
        end
        
    end
    
end
    
