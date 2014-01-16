#pragma once

#include <string>

class Control;
class XMLNode;

class Inflater
{	
public:
	static Control* Inflate(const std::string &xml);

private:
	static Control* LoadImageControl(XMLNode *node, const std::string &name);
	static Control* LoadAnimButtonControl(XMLNode *node, const std::string &name);
	static Control* LoadLabelControl(XMLNode *node, const std::string &name);
	static Control* LoadPanelControl(XMLNode *node, const std::string &name);
	static Control* LoadNode(XMLNode *node);
	static void LoadLayout(XMLNode *node, Control *control);
};

