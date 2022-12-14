#ifndef _CUST_BATTERY_METER_TABLE_LGC_H
#define _CUST_BATTERY_METER_TABLE_LGC_H

#define Q_MAX_POS_50_LGC		1868
#define Q_MAX_POS_25_LGC		1860
#define Q_MAX_POS_0_LGC			1450
#define Q_MAX_NEG_10_LGC		717

#define Q_MAX_POS_50_H_CURRENT_LGC	1855
#define Q_MAX_POS_25_H_CURRENT_LGC	1843
#define Q_MAX_POS_0_H_CURRENT_LGC	1290
#define Q_MAX_NEG_10_H_CURRENT_LGC	326

/* Battery profile data LG Chemical */

#define BATTERY_PROFILE_SIZE_LGC (sizeof(battery_profile_t2_LGC) / sizeof(BATTERY_PROFILE_STRUC))
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0_LGC[] =
{
	{0	,	4273},	// VC : 4273	0mA
	{4	,	4244},	// VC : 3800	30mA
	{8	,	4220},	// VC : 3759	60mA
	{13	,	4197},	// VC : 3731	90mA
	{17	,	4175},	// VC : 3706	119mA
	{21	,	4160},	// VC : 3678	149mA
	{25	,	4141},	// VC : 3645	179mA
	{29	,	4114},	// VC : 3607	209mA
	{33	,	4070},	// VC : 3558	239mA
	{38	,	4023},	// VC : 3462	269mA
	{42	,	3983},	// VC : 3367	298mA
	{46	,	3948},	// VC : 3295	328mA
	{50	,	3924},	// VC : 3242	358mA
	{54	,	3910},	// VC : 3215	388mA
	{58	,	3899},	// VC : 3200	418mA
	{62	,	3890},	// VC : 3199	443mA
	{65	,	3883},	// VC : 3200	464mA
	{68	,	3876},	// VC : 3200	484mA
	{70	,	3870},	// VC : 3199	502mA
	{72	,	3863},	// VC : 3199	519mA
	{74	,	3858},	// VC : 3199	533mA
	{76	,	3852},	// VC : 3199	547mA
	{78	,	3847},	// VC : 3200	559mA
	{79	,	3842},	// VC : 3199	570mA
	{81	,	3837},	// VC : 3199	579mA
	{82	,	3833},	// VC : 3200	588mA
	{83	,	3828},	// VC : 3199	596mA
	{84	,	3825},	// VC : 3199	604mA
	{85	,	3821},	// VC : 3199	610mA
	{86	,	3818},	// VC : 3199	617mA
	{87	,	3815},	// VC : 3199	622mA
	{88	,	3811},	// VC : 3199	628mA
	{88	,	3810},	// VC : 3199	632mA
	{89	,	3807},	// VC : 3199	637mA
	{89	,	3804},	// VC : 3199	641mA
	{90	,	3803},	// VC : 3198	645mA
	{90	,	3800},	// VC : 3199	648mA
	{91	,	3798},	// VC : 3199	652mA
	{91	,	3797},	// VC : 3200	655mA
	{92	,	3795},	// VC : 3198	658mA
	{92	,	3794},	// VC : 3200	660mA
	{92	,	3793},	// VC : 3199	662mA
	{93	,	3791},	// VC : 3199	665mA
	{93	,	3791},	// VC : 3199	667mA
	{93	,	3789},	// VC : 3199	669mA
	{94	,	3788},	// VC : 3198	671mA
	{94	,	3787},	// VC : 3199	673mA
	{94	,	3786},	// VC : 3199	675mA
	{94	,	3786},	// VC : 3197	676mA
	{95	,	3785},	// VC : 3199	678mA
	{95	,	3784},	// VC : 3199	679mA
	{95	,	3783},	// VC : 3198	681mA
	{95	,	3782},	// VC : 3198	682mA
	{95	,	3781},	// VC : 3199	683mA
	{96	,	3781},	// VC : 3200	685mA
	{96	,	3780},	// VC : 3198	686mA
	{96	,	3779},	// VC : 3197	687mA
	{96	,	3778},	// VC : 3199	689mA
	{96	,	3778},	// VC : 3198	690mA
	{96	,	3777},	// VC : 3196	691mA
	{97	,	3777},	// VC : 3200	692mA
	{97	,	3776},	// VC : 3200	693mA
	{97	,	3776},	// VC : 3200	695mA
	{97	,	3776},	// VC : 3199	696mA
	{97	,	3774},	// VC : 3199	697mA
	{97	,	3774},	// VC : 3200	698mA
	{97	,	3774},	// VC : 3199	699mA
	{98	,	3773},	// VC : 3196	700mA
	{98	,	3772},	// VC : 3200	701mA
	{98	,	3771},	// VC : 3198	701mA
	{98	,	3771},	// VC : 3199	702mA
	{98	,	3771},	// VC : 3200	703mA
	{98	,	3770},	// VC : 3198	704mA
	{98	,	3769},	// VC : 3197	705mA
	{98	,	3769},	// VC : 3198	706mA
	{98	,	3768},	// VC : 3196	706mA
	{99	,	3768},	// VC : 3197	707mA
	{99	,	3768},	// VC : 3199	708mA
	{99	,	3766},	// VC : 3196	708mA
	{99	,	3767},	// VC : 3195	709mA
	{99	,	3766},	// VC : 3197	710mA
	{99	,	3766},	// VC : 3199	711mA
	{99	,	3766},	// VC : 3198	711mA
	{99	,	3765},	// VC : 3199	712mA
	{99	,	3764},	// VC : 3198	713mA
	{99	,	3765},	// VC : 3199	713mA
	{100	,	3765},	// VC : 3200	714mA
	{100	,	3764},	// VC : 3200	714mA
	{100	,	3764},	// VC : 3198	715mA
	{100	,	3763},	// VC : 3197	716mA
	{100	,	3763},	// VC : 3200	716mA
	{100	,	3300},	// VC : 3200	717mA
};

// T1 0C
BATTERY_PROFILE_STRUC battery_profile_t1_LGC[] =
{
	{0	,	4323},	// VC : 4323	0mA
	{2	,	4298},	// VC : 4099	30mA
	{4	,	4275},	// VC : 4069	60mA
	{6	,	4254},	// VC : 4044	90mA
	{8	,	4231},	// VC : 4014	119mA
	{10	,	4199},	// VC : 3969	149mA
	{12	,	4159},	// VC : 3904	179mA
	{14	,	4126},	// VC : 3831	209mA
	{16	,	4103},	// VC : 3786	239mA
	{19	,	4088},	// VC : 3760	269mA
	{21	,	4074},	// VC : 3733	298mA
	{23	,	4055},	// VC : 3710	328mA
	{25	,	4024},	// VC : 3685	358mA
	{27	,	3995},	// VC : 3660	388mA
	{29	,	3974},	// VC : 3638	418mA
	{31	,	3958},	// VC : 3623	448mA
	{33	,	3945},	// VC : 3607	478mA
	{35	,	3934},	// VC : 3594	507mA
	{37	,	3921},	// VC : 3582	537mA
	{39	,	3908},	// VC : 3570	567mA
	{41	,	3895},	// VC : 3560	597mA
	{43	,	3883},	// VC : 3549	627mA
	{45	,	3873},	// VC : 3538	657mA
	{47	,	3862},	// VC : 3526	686mA
	{49	,	3853},	// VC : 3515	716mA
	{51	,	3845},	// VC : 3506	746mA
	{54	,	3837},	// VC : 3498	776mA
	{56	,	3830},	// VC : 3486	806mA
	{58	,	3823},	// VC : 3479	836mA
	{60	,	3816},	// VC : 3467	866mA
	{62	,	3809},	// VC : 3459	895mA
	{64	,	3801},	// VC : 3450	925mA
	{66	,	3795},	// VC : 3441	955mA
	{68	,	3788},	// VC : 3432	985mA
	{70	,	3779},	// VC : 3422	1015mA
	{72	,	3772},	// VC : 3412	1045mA
	{74	,	3762},	// VC : 3401	1074mA
	{76	,	3753},	// VC : 3391	1104mA
	{78	,	3744},	// VC : 3378	1134mA
	{80	,	3732},	// VC : 3365	1164mA
	{82	,	3723},	// VC : 3353	1194mA
	{84	,	3716},	// VC : 3341	1224mA
	{86	,	3711},	// VC : 3327	1254mA
	{88	,	3706},	// VC : 3306	1283mA
	{91	,	3699},	// VC : 3279	1313mA
	{93	,	3687},	// VC : 3238	1343mA
	{94	,	3665},	// VC : 3198	1370mA
	{96	,	3646},	// VC : 3200	1388mA
	{97	,	3627},	// VC : 3200	1401mA
	{97	,	3608},	// VC : 3200	1410mA
	{98	,	3592},	// VC : 3198	1418mA
	{98	,	3578},	// VC : 3199	1423mA
	{98	,	3568},	// VC : 3198	1428mA
	{99	,	3559},	// VC : 3200	1431mA
	{99	,	3552},	// VC : 3198	1434mA
	{99	,	3546},	// VC : 3199	1436mA
	{99	,	3543},	// VC : 3198	1438mA
	{99	,	3539},	// VC : 3198	1439mA
	{99	,	3536},	// VC : 3199	1440mA
	{99	,	3534},	// VC : 3198	1441mA
	{99	,	3531},	// VC : 3200	1442mA
	{100	,	3529},	// VC : 3199	1443mA
	{100	,	3527},	// VC : 3197	1444mA
	{100	,	3526},	// VC : 3198	1444mA
	{100	,	3524},	// VC : 3199	1445mA
	{100	,	3524},	// VC : 3198	1445mA
	{100	,	3522},	// VC : 3200	1446mA
	{100	,	3521},	// VC : 3198	1446mA
	{100	,	3520},	// VC : 3196	1446mA
	{100	,	3518},	// VC : 3197	1447mA
	{100	,	3517},	// VC : 3197	1447mA
	{100	,	3516},	// VC : 3198	1447mA
	{100	,	3515},	// VC : 3196	1447mA
	{100	,	3515},	// VC : 3198	1447mA
	{100	,	3513},	// VC : 3196	1448mA
	{100	,	3512},	// VC : 3199	1448mA
	{100	,	3511},	// VC : 3192	1448mA
	{100	,	3510},	// VC : 3199	1448mA
	{100	,	3510},	// VC : 3200	1448mA
	{100	,	3509},	// VC : 3195	1449mA
	{100	,	3507},	// VC : 3193	1449mA
	{100	,	3506},	// VC : 3192	1449mA
	{100	,	3505},	// VC : 3191	1449mA
	{100	,	3505},	// VC : 3198	1449mA
	{100	,	3504},	// VC : 3197	1449mA
	{100	,	3503},	// VC : 3189	1449mA
	{100	,	3502},	// VC : 3195	1449mA
	{100	,	3502},	// VC : 3187	1449mA
	{100	,	3500},	// VC : 3185	1450mA
	{100	,	3499},	// VC : 3185	1450mA
	{100	,	3498},	// VC : 3183	1450mA
	{100	,	3300},	// VC : 3198	1450mA
};

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2_LGC[] =
{
	{0	,	4330},	// VC : 4330	0mA
	{2	,	4307},	// VC : 4232	30mA
	{3	,	4286},	// VC : 4210	60mA
	{5	,	4266},	// VC : 4188	90mA
	{6	,	4248},	// VC : 4170	119mA
	{8	,	4229},	// VC : 4151	149mA
	{10	,	4211},	// VC : 4133	179mA
	{11	,	4194},	// VC : 4114	209mA
	{13	,	4175},	// VC : 4096	239mA
	{14	,	4158},	// VC : 4078	269mA
	{16	,	4141},	// VC : 4063	298mA
	{18	,	4124},	// VC : 4046	328mA
	{19	,	4107},	// VC : 4031	358mA
	{21	,	4091},	// VC : 4014	388mA
	{22	,	4075},	// VC : 3999	418mA
	{24	,	4061},	// VC : 3984	448mA
	{26	,	4051},	// VC : 3971	478mA
	{27	,	4029},	// VC : 3952	507mA
	{29	,	4009},	// VC : 3931	537mA
	{30	,	3998},	// VC : 3918	567mA
	{32	,	3988},	// VC : 3908	597mA
	{34	,	3975},	// VC : 3896	627mA
	{35	,	3963},	// VC : 3882	657mA
	{37	,	3951},	// VC : 3869	686mA
	{38	,	3939},	// VC : 3856	716mA
	{40	,	3927},	// VC : 3843	747mA
	{42	,	3912},	// VC : 3831	777mA
	{43	,	3895},	// VC : 3814	806mA
	{45	,	3879},	// VC : 3801	836mA
	{47	,	3866},	// VC : 3789	866mA
	{48	,	3856},	// VC : 3779	896mA
	{50	,	3847},	// VC : 3770	926mA
	{51	,	3838},	// VC : 3762	956mA
	{53	,	3831},	// VC : 3753	985mA
	{55	,	3824},	// VC : 3746	1015mA
	{56	,	3817},	// VC : 3739	1045mA
	{58	,	3811},	// VC : 3733	1075mA
	{59	,	3806},	// VC : 3727	1105mA
	{61	,	3801},	// VC : 3722	1135mA
	{63	,	3796},	// VC : 3717	1165mA
	{64	,	3791},	// VC : 3710	1194mA
	{66	,	3787},	// VC : 3706	1224mA
	{67	,	3783},	// VC : 3703	1254mA
	{69	,	3778},	// VC : 3698	1284mA
	{71	,	3773},	// VC : 3694	1314mA
	{72	,	3768},	// VC : 3691	1344mA
	{74	,	3761},	// VC : 3685	1373mA
	{75	,	3754},	// VC : 3680	1403mA
	{77	,	3747},	// VC : 3675	1433mA
	{79	,	3741},	// VC : 3668	1463mA
	{80	,	3735},	// VC : 3661	1493mA
	{82	,	3728},	// VC : 3653	1523mA
	{83	,	3717},	// VC : 3643	1553mA
	{85	,	3708},	// VC : 3633	1582mA
	{87	,	3695},	// VC : 3621	1612mA
	{88	,	3688},	// VC : 3616	1642mA
	{90	,	3686},	// VC : 3612	1672mA
	{92	,	3685},	// VC : 3607	1702mA
	{93	,	3681},	// VC : 3599	1732mA
	{95	,	3662},	// VC : 3581	1761mA
	{96	,	3601},	// VC : 3518	1791mA
	{98	,	3512},	// VC : 3419	1821mA
	{100	,	3366},	// VC : 3254	1851mA
	{100	,	3300},	// VC : 3200	1860mA
	{100	,	3281},	// VC : 3199	1863mA
	{100	,	3274},	// VC : 3199	1864mA
	{100	,	3270},	// VC : 3198	1864mA
	{100	,	3267},	// VC : 3197	1865mA
	{100	,	3266},	// VC : 3197	1865mA
	{100	,	3266},	// VC : 3198	1865mA
	{100	,	3265},	// VC : 3198	1865mA
	{100	,	3263},	// VC : 3198	1865mA
	{100	,	3263},	// VC : 3198	1865mA
	{100	,	3264},	// VC : 3198	1865mA
	{100	,	3261},	// VC : 3196	1866mA
	{100	,	3259},	// VC : 3197	1866mA
	{100	,	3259},	// VC : 3197	1866mA
	{100	,	3257},	// VC : 3194	1866mA
	{100	,	3254},	// VC : 3192	1866mA
	{100	,	3254},	// VC : 3190	1866mA
	{100	,	3254},	// VC : 3190	1866mA
	{100	,	3253},	// VC : 3189	1866mA
	{100	,	3253},	// VC : 3189	1866mA
	{100	,	3250},	// VC : 3186	1866mA
	{100	,	3247},	// VC : 3184	1867mA
	{100	,	3243},	// VC : 3181	1867mA
	{100	,	3240},	// VC : 3181	1867mA
	{100	,	3238},	// VC : 3180	1867mA
	{100	,	3237},	// VC : 3179	1867mA
	{100	,	3234},	// VC : 3176	1867mA
	{100	,	3234},	// VC : 3176	1867mA
	{100	,	3234},	// VC : 3173	1867mA
};

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3_LGC[] =
{
	{0	,	4343},	// VC : 4343	0mA
	{2	,	4320},	// VC : 4259	30mA
	{3	,	4300},	// VC : 4239	60mA
	{5	,	4279},	// VC : 4218	90mA
	{6	,	4260},	// VC : 4199	119mA
	{8	,	4241},	// VC : 4180	149mA
	{10	,	4224},	// VC : 4162	179mA
	{11	,	4205},	// VC : 4143	209mA
	{13	,	4187},	// VC : 4126	239mA
	{14	,	4171},	// VC : 4108	269mA
	{16	,	4153},	// VC : 4091	298mA
	{18	,	4136},	// VC : 4072	328mA
	{19	,	4119},	// VC : 4055	358mA
	{21	,	4104},	// VC : 4039	388mA
	{22	,	4087},	// VC : 4022	418mA
	{24	,	4071},	// VC : 4005	448mA
	{26	,	4055},	// VC : 3989	478mA
	{27	,	4041},	// VC : 3973	507mA
	{29	,	4026},	// VC : 3958	537mA
	{30	,	4012},	// VC : 3944	567mA
	{32	,	3999},	// VC : 3929	597mA
	{34	,	3985},	// VC : 3916	627mA
	{35	,	3972},	// VC : 3901	657mA
	{37	,	3960},	// VC : 3887	686mA
	{38	,	3948},	// VC : 3873	716mA
	{40	,	3936},	// VC : 3860	746mA
	{42	,	3923},	// VC : 3847	776mA
	{43	,	3907},	// VC : 3834	806mA
	{45	,	3889},	// VC : 3821	836mA
	{46	,	3874},	// VC : 3809	866mA
	{48	,	3863},	// VC : 3798	895mA
	{50	,	3853},	// VC : 3789	925mA
	{51	,	3844},	// VC : 3781	955mA
	{53	,	3836},	// VC : 3772	985mA
	{54	,	3829},	// VC : 3764	1015mA
	{56	,	3821},	// VC : 3757	1045mA
	{57	,	3815},	// VC : 3751	1074mA
	{59	,	3809},	// VC : 3743	1104mA
	{61	,	3803},	// VC : 3736	1134mA
	{62	,	3798},	// VC : 3731	1164mA
	{64	,	3794},	// VC : 3725	1194mA
	{66	,	3788},	// VC : 3719	1224mA
	{67	,	3784},	// VC : 3714	1254mA
	{69	,	3779},	// VC : 3708	1283mA
	{70	,	3773},	// VC : 3704	1313mA
	{72	,	3761},	// VC : 3697	1343mA
	{74	,	3752},	// VC : 3688	1373mA
	{75	,	3746},	// VC : 3681	1403mA
	{77	,	3738},	// VC : 3674	1433mA
	{78	,	3732},	// VC : 3668	1462mA
	{80	,	3725},	// VC : 3661	1492mA
	{81	,	3719},	// VC : 3656	1522mA
	{83	,	3710},	// VC : 3646	1552mA
	{85	,	3700},	// VC : 3636	1582mA
	{86	,	3690},	// VC : 3625	1612mA
	{88	,	3678},	// VC : 3616	1642mA
	{89	,	3676},	// VC : 3613	1671mA
	{91	,	3674},	// VC : 3609	1701mA
	{93	,	3672},	// VC : 3604	1731mA
	{94	,	3662},	// VC : 3594	1761mA
	{96	,	3613},	// VC : 3543	1791mA
	{97	,	3536},	// VC : 3460	1821mA
	{99	,	3416},	// VC : 3334	1850mA
	{100	,	3289},	// VC : 3199	1870mA
	{100	,	3268},	// VC : 3199	1872mA
	{100	,	3261},	// VC : 3198	1873mA
	{100	,	3260},	// VC : 3199	1874mA
	{100	,	3257},	// VC : 3199	1874mA
	{100	,	3255},	// VC : 3197	1874mA
	{100	,	3254},	// VC : 3198	1874mA
	{100	,	3253},	// VC : 3196	1874mA
	{100	,	3253},	// VC : 3199	1874mA
	{100	,	3252},	// VC : 3197	1874mA
	{100	,	3251},	// VC : 3197	1874mA
	{100	,	3250},	// VC : 3195	1874mA
	{100	,	3249},	// VC : 3194	1874mA
	{100	,	3249},	// VC : 3195	1874mA
	{100	,	3248},	// VC : 3194	1874mA
	{100	,	3247},	// VC : 3193	1874mA
	{100	,	3246},	// VC : 3191	1874mA
	{100	,	3244},	// VC : 3190	1874mA
	{100	,	3242},	// VC : 3189	1874mA
	{100	,	3241},	// VC : 3187	1874mA
	{100	,	3240},	// VC : 3187	1874mA
	{100	,	3238},	// VC : 3183	1874mA
	{100	,	3237},	// VC : 3183	1874mA
	{100	,	3236},	// VC : 3183	1874mA
	{100	,	3235},	// VC : 3182	1874mA
	{100	,	3233},	// VC : 3180	1874mA
	{100	,	3232},	// VC : 3178	1874mA
	{100	,	3230},	// VC : 3175	1874mA
	{100	,	3230},	// VC : 3173	1874mA
};

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature_LGC[] =
{
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
};

#define R_PROFILE_SIZE_LGC (sizeof(r_profile_t2_LGC) / sizeof(R_PROFILE_STRUC))
// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0_LGC[] =
{
	{1110	,	4273},		// VC : 4273	0mA
	{1110	,	4244},		// VC : 3800	30mA
	{1153	,	4220},		// VC : 3759	60mA
	{1165	,	4197},		// VC : 3731	90mA
	{1173	,	4175},		// VC : 3706	119mA
	{1205	,	4160},		// VC : 3678	149mA
	{1240	,	4141},		// VC : 3645	179mA
	{1268	,	4114},		// VC : 3607	209mA
	{1280	,	4070},		// VC : 3558	239mA
	{1403	,	4023},		// VC : 3462	269mA
	{1540	,	3983},		// VC : 3367	298mA
	{1633	,	3948},		// VC : 3295	328mA
	{1705	,	3924},		// VC : 3242	358mA
	{1738	,	3910},		// VC : 3215	388mA
	{1748	,	3899},		// VC : 3200	418mA
	{1728	,	3890},		// VC : 3199	443mA
	{1708	,	3883},		// VC : 3200	464mA
	{1690	,	3876},		// VC : 3200	484mA
	{1678	,	3870},		// VC : 3199	502mA
	{1660	,	3863},		// VC : 3199	519mA
	{1648	,	3858},		// VC : 3199	533mA
	{1633	,	3852},		// VC : 3199	547mA
	{1618	,	3847},		// VC : 3200	559mA
	{1608	,	3842},		// VC : 3199	570mA
	{1595	,	3837},		// VC : 3199	579mA
	{1583	,	3833},		// VC : 3200	588mA
	{1573	,	3828},		// VC : 3199	596mA
	{1565	,	3825},		// VC : 3199	604mA
	{1555	,	3821},		// VC : 3199	610mA
	{1548	,	3818},		// VC : 3199	617mA
	{1540	,	3815},		// VC : 3199	622mA
	{1530	,	3811},		// VC : 3199	628mA
	{1528	,	3810},		// VC : 3199	632mA
	{1520	,	3807},		// VC : 3199	637mA
	{1513	,	3804},		// VC : 3199	641mA
	{1513	,	3803},		// VC : 3198	645mA
	{1503	,	3800},		// VC : 3199	648mA
	{1498	,	3798},		// VC : 3199	652mA
	{1493	,	3797},		// VC : 3200	655mA
	{1493	,	3795},		// VC : 3198	658mA
	{1485	,	3794},		// VC : 3200	660mA
	{1485	,	3793},		// VC : 3199	662mA
	{1480	,	3791},		// VC : 3199	665mA
	{1480	,	3791},		// VC : 3199	667mA
	{1475	,	3789},		// VC : 3199	669mA
	{1475	,	3788},		// VC : 3198	671mA
	{1470	,	3787},		// VC : 3199	673mA
	{1468	,	3786},		// VC : 3199	675mA
	{1473	,	3786},		// VC : 3197	676mA
	{1465	,	3785},		// VC : 3199	678mA
	{1463	,	3784},		// VC : 3199	679mA
	{1463	,	3783},		// VC : 3198	681mA
	{1460	,	3782},		// VC : 3198	682mA
	{1455	,	3781},		// VC : 3199	683mA
	{1453	,	3781},		// VC : 3200	685mA
	{1455	,	3780},		// VC : 3198	686mA
	{1455	,	3779},		// VC : 3197	687mA
	{1448	,	3778},		// VC : 3199	689mA
	{1450	,	3778},		// VC : 3198	690mA
	{1453	,	3777},		// VC : 3196	691mA
	{1443	,	3777},		// VC : 3200	692mA
	{1440	,	3776},		// VC : 3200	693mA
	{1440	,	3776},		// VC : 3200	695mA
	{1443	,	3776},		// VC : 3199	696mA
	{1438	,	3774},		// VC : 3199	697mA
	{1435	,	3774},		// VC : 3200	698mA
	{1438	,	3774},		// VC : 3199	699mA
	{1443	,	3773},		// VC : 3196	700mA
	{1430	,	3772},		// VC : 3200	701mA
	{1433	,	3771},		// VC : 3198	701mA
	{1430	,	3771},		// VC : 3199	702mA
	{1428	,	3771},		// VC : 3200	703mA
	{1430	,	3770},		// VC : 3198	704mA
	{1430	,	3769},		// VC : 3197	705mA
	{1428	,	3769},		// VC : 3198	706mA
	{1430	,	3768},		// VC : 3196	706mA
	{1428	,	3768},		// VC : 3197	707mA
	{1423	,	3768},		// VC : 3199	708mA
	{1425	,	3766},		// VC : 3196	708mA
	{1430	,	3767},		// VC : 3195	709mA
	{1423	,	3766},		// VC : 3197	710mA
	{1418	,	3766},		// VC : 3199	711mA
	{1420	,	3766},		// VC : 3198	711mA
	{1415	,	3765},		// VC : 3199	712mA
	{1415	,	3764},		// VC : 3198	713mA
	{1415	,	3765},		// VC : 3199	713mA
	{1413	,	3765},		// VC : 3200	714mA
	{1410	,	3764},		// VC : 3200	714mA
	{1415	,	3764},		// VC : 3198	715mA
	{1415	,	3763},		// VC : 3197	716mA
	{1408	,	3763},		// VC : 3200	716mA
	{250	,	3300},		// VC : 3200	717mA
};

// T1 0C
R_PROFILE_STRUC r_profile_t1_LGC[] =
{
	{498	,	4323},		// VC : 4323	0mA
	{498	,	4298},		// VC : 4099	30mA
	{515	,	4275},		// VC : 4069	60mA
	{525	,	4254},		// VC : 4044	90mA
	{543	,	4231},		// VC : 4014	119mA
	{575	,	4199},		// VC : 3969	149mA
	{638	,	4159},		// VC : 3904	179mA
	{738	,	4126},		// VC : 3831	209mA
	{793	,	4103},		// VC : 3786	239mA
	{820	,	4088},		// VC : 3760	269mA
	{853	,	4074},		// VC : 3733	298mA
	{863	,	4055},		// VC : 3710	328mA
	{848	,	4024},		// VC : 3685	358mA
	{838	,	3995},		// VC : 3660	388mA
	{840	,	3974},		// VC : 3638	418mA
	{838	,	3958},		// VC : 3623	448mA
	{845	,	3945},		// VC : 3607	478mA
	{850	,	3934},		// VC : 3594	507mA
	{848	,	3921},		// VC : 3582	537mA
	{845	,	3908},		// VC : 3570	567mA
	{838	,	3895},		// VC : 3560	597mA
	{835	,	3883},		// VC : 3549	627mA
	{838	,	3873},		// VC : 3538	657mA
	{840	,	3862},		// VC : 3526	686mA
	{845	,	3853},		// VC : 3515	716mA
	{848	,	3845},		// VC : 3506	746mA
	{848	,	3837},		// VC : 3498	776mA
	{860	,	3830},		// VC : 3486	806mA
	{860	,	3823},		// VC : 3479	836mA
	{873	,	3816},		// VC : 3467	866mA
	{875	,	3809},		// VC : 3459	895mA
	{878	,	3801},		// VC : 3450	925mA
	{885	,	3795},		// VC : 3441	955mA
	{890	,	3788},		// VC : 3432	985mA
	{893	,	3779},		// VC : 3422	1015mA
	{900	,	3772},		// VC : 3412	1045mA
	{903	,	3762},		// VC : 3401	1074mA
	{905	,	3753},		// VC : 3391	1104mA
	{915	,	3744},		// VC : 3378	1134mA
	{918	,	3732},		// VC : 3365	1164mA
	{925	,	3723},		// VC : 3353	1194mA
	{938	,	3716},		// VC : 3341	1224mA
	{960	,	3711},		// VC : 3327	1254mA
	{1000	,	3706},		// VC : 3306	1283mA
	{1050	,	3699},		// VC : 3279	1313mA
	{1123	,	3687},		// VC : 3238	1343mA
	{1168	,	3665},		// VC : 3198	1370mA
	{1115	,	3646},		// VC : 3200	1388mA
	{1068	,	3627},		// VC : 3200	1401mA
	{1020	,	3608},		// VC : 3200	1410mA
	{985	,	3592},		// VC : 3198	1418mA
	{948	,	3578},		// VC : 3199	1423mA
	{925	,	3568},		// VC : 3198	1428mA
	{898	,	3559},		// VC : 3200	1431mA
	{885	,	3552},		// VC : 3198	1434mA
	{868	,	3546},		// VC : 3199	1436mA
	{863	,	3543},		// VC : 3198	1438mA
	{853	,	3539},		// VC : 3198	1439mA
	{843	,	3536},		// VC : 3199	1440mA
	{840	,	3534},		// VC : 3198	1441mA
	{828	,	3531},		// VC : 3200	1442mA
	{825	,	3529},		// VC : 3199	1443mA
	{825	,	3527},		// VC : 3197	1444mA
	{820	,	3526},		// VC : 3198	1444mA
	{813	,	3524},		// VC : 3199	1445mA
	{815	,	3524},		// VC : 3198	1445mA
	{805	,	3522},		// VC : 3200	1446mA
	{808	,	3521},		// VC : 3198	1446mA
	{810	,	3520},		// VC : 3196	1446mA
	{803	,	3518},		// VC : 3197	1447mA
	{800	,	3517},		// VC : 3197	1447mA
	{795	,	3516},		// VC : 3198	1447mA
	{798	,	3515},		// VC : 3196	1447mA
	{793	,	3515},		// VC : 3198	1447mA
	{793	,	3513},		// VC : 3196	1448mA
	{783	,	3512},		// VC : 3199	1448mA
	{798	,	3511},		// VC : 3192	1448mA
	{778	,	3510},		// VC : 3199	1448mA
	{775	,	3510},		// VC : 3200	1448mA
	{785	,	3509},		// VC : 3195	1449mA
	{785	,	3507},		// VC : 3193	1449mA
	{785	,	3506},		// VC : 3192	1449mA
	{785	,	3505},		// VC : 3191	1449mA
	{768	,	3505},		// VC : 3198	1449mA
	{768	,	3504},		// VC : 3197	1449mA
	{785	,	3503},		// VC : 3189	1449mA
	{768	,	3502},		// VC : 3195	1449mA
	{788	,	3502},		// VC : 3187	1449mA
	{788	,	3500},		// VC : 3185	1450mA
	{785	,	3499},		// VC : 3185	1450mA
	{788	,	3498},		// VC : 3183	1450mA
	{255	,	3300},		// VC : 3198	1450mA
};

// T2 25C
R_PROFILE_STRUC r_profile_t2_LGC[] =
{
	{188	,	4330},		// VC : 4330	0mA
	{188	,	4307},		// VC : 4232	30mA
	{190	,	4286},		// VC : 4210	60mA
	{195	,	4266},		// VC : 4188	90mA
	{195	,	4248},		// VC : 4170	119mA
	{195	,	4229},		// VC : 4151	149mA
	{195	,	4211},		// VC : 4133	179mA
	{200	,	4194},		// VC : 4114	209mA
	{198	,	4175},		// VC : 4096	239mA
	{200	,	4158},		// VC : 4078	269mA
	{195	,	4141},		// VC : 4063	298mA
	{195	,	4124},		// VC : 4046	328mA
	{190	,	4107},		// VC : 4031	358mA
	{193	,	4091},		// VC : 4014	388mA
	{190	,	4075},		// VC : 3999	418mA
	{193	,	4061},		// VC : 3984	448mA
	{200	,	4051},		// VC : 3971	478mA
	{193	,	4029},		// VC : 3952	507mA
	{195	,	4009},		// VC : 3931	537mA
	{200	,	3998},		// VC : 3918	567mA
	{200	,	3988},		// VC : 3908	597mA
	{198	,	3975},		// VC : 3896	627mA
	{203	,	3963},		// VC : 3882	657mA
	{205	,	3951},		// VC : 3869	686mA
	{208	,	3939},		// VC : 3856	716mA
	{210	,	3927},		// VC : 3843	747mA
	{203	,	3912},		// VC : 3831	777mA
	{203	,	3895},		// VC : 3814	806mA
	{195	,	3879},		// VC : 3801	836mA
	{193	,	3866},		// VC : 3789	866mA
	{193	,	3856},		// VC : 3779	896mA
	{193	,	3847},		// VC : 3770	926mA
	{190	,	3838},		// VC : 3762	956mA
	{195	,	3831},		// VC : 3753	985mA
	{195	,	3824},		// VC : 3746	1015mA
	{195	,	3817},		// VC : 3739	1045mA
	{195	,	3811},		// VC : 3733	1075mA
	{198	,	3806},		// VC : 3727	1105mA
	{198	,	3801},		// VC : 3722	1135mA
	{198	,	3796},		// VC : 3717	1165mA
	{203	,	3791},		// VC : 3710	1194mA
	{203	,	3787},		// VC : 3706	1224mA
	{200	,	3783},		// VC : 3703	1254mA
	{200	,	3778},		// VC : 3698	1284mA
	{198	,	3773},		// VC : 3694	1314mA
	{193	,	3768},		// VC : 3691	1344mA
	{190	,	3761},		// VC : 3685	1373mA
	{185	,	3754},		// VC : 3680	1403mA
	{180	,	3747},		// VC : 3675	1433mA
	{183	,	3741},		// VC : 3668	1463mA
	{185	,	3735},		// VC : 3661	1493mA
	{188	,	3728},		// VC : 3653	1523mA
	{185	,	3717},		// VC : 3643	1553mA
	{188	,	3708},		// VC : 3633	1582mA
	{185	,	3695},		// VC : 3621	1612mA
	{180	,	3688},		// VC : 3616	1642mA
	{185	,	3686},		// VC : 3612	1672mA
	{195	,	3685},		// VC : 3607	1702mA
	{205	,	3681},		// VC : 3599	1732mA
	{203	,	3662},		// VC : 3581	1761mA
	{208	,	3601},		// VC : 3518	1791mA
	{233	,	3512},		// VC : 3419	1821mA
	{280	,	3366},		// VC : 3254	1851mA
	{250	,	3300},		// VC : 3200	1860mA
	{205	,	3281},		// VC : 3199	1863mA
	{188	,	3274},		// VC : 3199	1864mA
	{180	,	3270},		// VC : 3198	1864mA
	{175	,	3267},		// VC : 3197	1865mA
	{173	,	3266},		// VC : 3197	1865mA
	{170	,	3266},		// VC : 3198	1865mA
	{168	,	3265},		// VC : 3198	1865mA
	{163	,	3263},		// VC : 3198	1865mA
	{163	,	3263},		// VC : 3198	1865mA
	{165	,	3264},		// VC : 3198	1865mA
	{163	,	3261},		// VC : 3196	1866mA
	{155	,	3259},		// VC : 3197	1866mA
	{155	,	3259},		// VC : 3197	1866mA
	{158	,	3257},		// VC : 3194	1866mA
	{155	,	3254},		// VC : 3192	1866mA
	{160	,	3254},		// VC : 3190	1866mA
	{160	,	3254},		// VC : 3190	1866mA
	{160	,	3253},		// VC : 3189	1866mA
	{160	,	3253},		// VC : 3189	1866mA
	{160	,	3250},		// VC : 3186	1866mA
	{158	,	3247},		// VC : 3184	1867mA
	{155	,	3243},		// VC : 3181	1867mA
	{148	,	3240},		// VC : 3181	1867mA
	{145	,	3238},		// VC : 3180	1867mA
	{145	,	3237},		// VC : 3179	1867mA
	{145	,	3234},		// VC : 3176	1867mA
	{145	,	3234},		// VC : 3176	1867mA
	{153	,	3234},		// VC : 3173	1867mA
};

// T3 50C
R_PROFILE_STRUC r_profile_t3_LGC[] =
{
	{153	,	4343},		// VC : 4343	0mA
	{153	,	4320},		// VC : 4259	30mA
	{153	,	4300},		// VC : 4239	60mA
	{153	,	4279},		// VC : 4218	90mA
	{153	,	4260},		// VC : 4199	119mA
	{153	,	4241},		// VC : 4180	149mA
	{155	,	4224},		// VC : 4162	179mA
	{155	,	4205},		// VC : 4143	209mA
	{153	,	4187},		// VC : 4126	239mA
	{158	,	4171},		// VC : 4108	269mA
	{155	,	4153},		// VC : 4091	298mA
	{160	,	4136},		// VC : 4072	328mA
	{160	,	4119},		// VC : 4055	358mA
	{163	,	4104},		// VC : 4039	388mA
	{163	,	4087},		// VC : 4022	418mA
	{165	,	4071},		// VC : 4005	448mA
	{165	,	4055},		// VC : 3989	478mA
	{170	,	4041},		// VC : 3973	507mA
	{170	,	4026},		// VC : 3958	537mA
	{170	,	4012},		// VC : 3944	567mA
	{175	,	3999},		// VC : 3929	597mA
	{173	,	3985},		// VC : 3916	627mA
	{178	,	3972},		// VC : 3901	657mA
	{183	,	3960},		// VC : 3887	686mA
	{188	,	3948},		// VC : 3873	716mA
	{190	,	3936},		// VC : 3860	746mA
	{190	,	3923},		// VC : 3847	776mA
	{183	,	3907},		// VC : 3834	806mA
	{170	,	3889},		// VC : 3821	836mA
	{163	,	3874},		// VC : 3809	866mA
	{163	,	3863},		// VC : 3798	895mA
	{160	,	3853},		// VC : 3789	925mA
	{158	,	3844},		// VC : 3781	955mA
	{160	,	3836},		// VC : 3772	985mA
	{163	,	3829},		// VC : 3764	1015mA
	{160	,	3821},		// VC : 3757	1045mA
	{160	,	3815},		// VC : 3751	1074mA
	{165	,	3809},		// VC : 3743	1104mA
	{168	,	3803},		// VC : 3736	1134mA
	{168	,	3798},		// VC : 3731	1164mA
	{173	,	3794},		// VC : 3725	1194mA
	{173	,	3788},		// VC : 3719	1224mA
	{175	,	3784},		// VC : 3714	1254mA
	{178	,	3779},		// VC : 3708	1283mA
	{173	,	3773},		// VC : 3704	1313mA
	{160	,	3761},		// VC : 3697	1343mA
	{160	,	3752},		// VC : 3688	1373mA
	{163	,	3746},		// VC : 3681	1403mA
	{160	,	3738},		// VC : 3674	1433mA
	{160	,	3732},		// VC : 3668	1462mA
	{160	,	3725},		// VC : 3661	1492mA
	{158	,	3719},		// VC : 3656	1522mA
	{160	,	3710},		// VC : 3646	1552mA
	{160	,	3700},		// VC : 3636	1582mA
	{163	,	3690},		// VC : 3625	1612mA
	{155	,	3678},		// VC : 3616	1642mA
	{158	,	3676},		// VC : 3613	1671mA
	{163	,	3674},		// VC : 3609	1701mA
	{170	,	3672},		// VC : 3604	1731mA
	{170	,	3662},		// VC : 3594	1761mA
	{175	,	3613},		// VC : 3543	1791mA
	{190	,	3536},		// VC : 3460	1821mA
	{205	,	3416},		// VC : 3334	1850mA
	{225	,	3289},		// VC : 3199	1870mA
	{173	,	3268},		// VC : 3199	1872mA
	{158	,	3261},		// VC : 3198	1873mA
	{153	,	3260},		// VC : 3199	1874mA
	{145	,	3257},		// VC : 3199	1874mA
	{145	,	3255},		// VC : 3197	1874mA
	{140	,	3254},		// VC : 3198	1874mA
	{143	,	3253},		// VC : 3196	1874mA
	{135	,	3253},		// VC : 3199	1874mA
	{138	,	3252},		// VC : 3197	1874mA
	{135	,	3251},		// VC : 3197	1874mA
	{138	,	3250},		// VC : 3195	1874mA
	{138	,	3249},		// VC : 3194	1874mA
	{135	,	3249},		// VC : 3195	1874mA
	{135	,	3248},		// VC : 3194	1874mA
	{135	,	3247},		// VC : 3193	1874mA
	{138	,	3246},		// VC : 3191	1874mA
	{135	,	3244},		// VC : 3190	1874mA
	{133	,	3242},		// VC : 3189	1874mA
	{135	,	3241},		// VC : 3187	1874mA
	{133	,	3240},		// VC : 3187	1874mA
	{138	,	3238},		// VC : 3183	1874mA
	{135	,	3237},		// VC : 3183	1874mA
	{133	,	3236},		// VC : 3183	1874mA
	{133	,	3235},		// VC : 3182	1874mA
	{133	,	3233},		// VC : 3180	1874mA
	{135	,	3232},		// VC : 3178	1874mA
	{138	,	3230},		// VC : 3175	1874mA
	{143	,	3230},		// VC : 3173	1874mA
};

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature_LGC[] =
{
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
	{0	,	0},
};

#endif /* _CUST_BATTERY_METER_TABLE_LGC_H */
