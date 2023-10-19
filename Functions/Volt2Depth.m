function depth = Volt2Depth(volt,linearmodel)
%VOLT2DEPTH ���[�U�[�Z���T����̓d���f�[�^��\������Ă��鐔�l�ɕϊ�����B
%   �d���ƕ\���̐��l�ɂ͌덷�����݂��A���̌덷�͓d���̑傫���ɔ�Ⴗ��B���̊֐�
%   �ł��̌덷���C������B
p1 = linearmodel.p1;
p2 = linearmodel.p2;
volt = (volt-2.5)*2;
depth = volt + p1*volt + p2;
end

