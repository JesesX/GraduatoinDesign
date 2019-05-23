tic

% % ������Ʒ���
% subsystemTypes = {'QPSK 1/4', ...
%                     'QPSK 1/3', 'QPSK 2/5', 'QPSK 1/2', 'QPSK 3/5', 'QPSK 2/3', ...
%                     'QPSK 3/4', 'QPSK 4/5', 'QPSK 5/6', 'QPSK 8/9', 'QPSK 9/10',...
%                     '8PSK 3/5', '8PSK 4/5', '8PSK 2/3', '8PSK 3/4', '8PSK 5/6', ...
%                     '8PSK 8/9', '8PSK 9/10', '16APSK 2/3', '16APSK 3/4', ...
%                     '16APSK 4/5', '16APSK 5/6', '16APSK 8/9', '16APSK 9/10', ...
%                     '32APSK 3/4', '32APSK 4/5', '32APSK 5/6', '32APSK 8/9', ...
%                     '32APSK 9/10'}

subsystemTypes = { '8PSK 3/4'}

EsNodBs       = 5 : 0.01 : 11                                            %�����(dB)
numFrames     = 700;                                                         %֡�ĸ���
BerLDPCs      = zeros(numel(subsystemTypes), numel(EsNodBs))               %��ʼ��������


for j = 1 : numel(subsystemTypes)
    subsystemType = subsystemTypes{j};
    
    for i = 1 : numel(EsNodBs)
        EsNodB = EsNodBs(i);  
        
        % ��ʼ��
        configureDVBS2Demo

        % Display system parameters
        dvb
        fprintf('Is doing subsystemType: %d\n', j)
        fprintf('Will do EsNodB: %d\n', i)

        
        encldpc = comm.LDPCEncoder(dvb.LDPCParityCheckMatrix);

        decldpc = comm.LDPCDecoder(dvb.LDPCParityCheckMatrix, ...
            'IterationTerminationCondition', 'Parity check satisfied', ...
            'MaximumIterationCount',         dvb.LDPCNumIterations, ...
            'NumIterationsOutputPort',       true);

        bbFrameTx  = false(encbch.MessageLength,1);
        numIterVec = zeros(numFrames, 1);
        falseVec   = false(dvb.NumPacketsPerBBFrame, 1);

        for frameCnt=1:numFrames

            bbFrameTx(1:dvb.NumInfoBitsPerCodeword) = ...
                  logical(randi([0 1], dvb.NumInfoBitsPerCodeword, 1));

            bchEncOut = encbch(bbFrameTx);                                  %bch����
            ldpcEncOut = encldpc(bchEncOut);                                %ldpc����
            intrlvrOut = intrlvr(ldpcEncOut);                               %��֯

            if dvb.ModulationOrder == 4 || dvb.ModulationOrder == 8         %psk����
                modOut = pskModulator(intrlvrOut);
            else
                modOut = dvbsapskmod(intrlvrOut, dvb.ModulationOrder, 's2', ...
                    dvb.CodeRate, 'InputType', 'bit', 'UnitAveragePower', true);    %apsk����
            end

            chanOut = chan(modOut);

            if dvb.ModulationOrder == 4 || dvb.ModulationOrder == 8
                demodOut = pskDemodulator(chanOut);                         %psk���
            else
                demodOut = dvbsapskdemod(chanOut, dvb.ModulationOrder, 's2', ...
                    dvb.CodeRate, 'OutputType', 'approxllr', 'NoiseVar', ...
                    dvb.NoiseVar, 'UnitAveragePower', true);                %apsk���
            end

            deintrlvrOut = deintrlvr(demodOut);                             %�⽻֯
            [ldpcDecOut, numIter] = decldpc(deintrlvrOut);                  %ldpc����
            bchDecOut = decbch(ldpcDecOut);                                 %bch����
            bbFrameRx = bchDecOut(1:dvb.NumInfoBitsPerCodeword,1);    

            comparedBits = xor(bbFrameRx, bbFrameTx(1:dvb.NumInfoBitsPerCodeword));
            packetErr    = any(reshape(comparedBits, dvb.NumBitsPerPacket, ...
                               dvb.NumPacketsPerBBFrame));
            per = PER(falseVec,   packetErr');    
            berMod = BERMod(demodOut<0, intrlvrOut);
            berLDPC = BERLDPC(ldpcDecOut, bchEncOut);
            
            if frameCnt == numFrames && i == numel(EsNodBs)
                BerLDPCss = berLDPC(1)
            end
            
            BerLDPCs(j, i) = BerLDPCs(j, i) + berLDPC(1);                   %����������
            numIterVec(frameCnt) = numIter;
            noiseVar   = meanCalc(varCalc(chanOut - modOut));

        end
        BerLDPCs(j, i) = BerLDPCs(j, i) / numFrames;
    end
    BerLDPCs(j, :)


    figure(1)
    hold on
    semilogy(EsNodBs, BerLDPCs(j, :));
    legend(subsystemTypes);
    xlabel('SNR (dB)'); ylabel('BER'); grid on

 
end


figure(2)
semilogy(EsNodBs, BerLDPCs);
legend(subsystemTypes);
xlabel('SNR (dB)'); ylabel('BER'); grid on
    
save('F:\Corrosponding_Engineering\��ҵ���\matlab\8PSK\8PSK4.mat', 'subsystemTypes', 'EsNodBs', 'BerLDPCs')

displayEndOfDemoMessage(mfilename)

load train
sound(y,Fs)

toc