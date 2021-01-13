/* Quartus Prime Version 18.1.0 Build 625 09/12/2018 SJ Lite Edition */
JedecChain;
	FileRevision(JESD32A);
	DefaultMfr(6E);

	P ActionCode(Ign)
		Device PartName(SOCVHPS) MfrSpec(OpMask(0));
	P ActionCode(Cfg)
		Device PartName(5CSEBA6U23) Path("/home/patmos/t-crest/patmos/hardware/quartus/de10-nano-drone/") File("patmos.sof") MfrSpec(OpMask(1));

ChainEnd;

AlteraBegin;
	ChainType(JTAG);
AlteraEnd;
