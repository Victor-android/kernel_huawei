/* < DTS2011090704268 jiaxianghong 20110914 begin */

��֧���ֹ����ش˼�⹦�ܡ�
1.��adb shell�����ֻ�����ʹ��insmod�������rsm.koģ�飬�������rsmĬ�ϻ�����н��̽��м�أ�Ĭ�ϼ��ʱ����Ϊ120s��
2.����ģ�����/proc/rsm/Ŀ¼������pname��timeout�����ļ���ʹ�������������в��������ã�
echo xx > pname ���ò���Ҫ��صĽ�������
echo xx > timeout ���ü�صļ��ʱ��
�������ų���صĽ�����Ϊ16����
���粻��Ҫ���rpcrotuer_smd_x��krmt_storagecln��krmt_storagecln��krtcclntd��krtcclntcbd��kbatteryclntd��kbatteryclntcbd�Ƚ��̣�������̼��ö��Ż��߿ո�����������ʱ����Ϊ60s��
echo rpcrotuer_smd_x��krmt_storagecln��krmt_storagecln��krtcclntd��krtcclntcbd��kbatteryclntd��kbatteryclntcbd > pname
echo 60 > timeout

/* DTS2011090704268 jiaxianghong 20110914 end > */