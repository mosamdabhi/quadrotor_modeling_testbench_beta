function q = ZYXToQuat(eulerAngles)
R = ZYXToR(eulerAngles);
q = RToQuat(R);