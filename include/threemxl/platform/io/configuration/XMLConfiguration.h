#ifndef XMLCONFIGURATION_H_
#define XMLCONFIGURATION_H_

#include <string>
//#include <map>

//#include <utilities/TextFile.hpp>
#include "Configuration.h"
#include <tinyxml.h>
#include <vector>

// Forward declaration
class IXMLConfigProperty;

class IXMLConfigSection: public IConfigSection
{
		friend class CConfigSection;

	private:
		TiXmlElement*		mPElement;
		bool				isSection(TiXmlElement* pElement) const;
		bool				isProperty(TiXmlElement* pElement) const;

	protected:
		// Make the interface protected so that noone will accidentally use this class
		std::string			name() const;
		bool 				hasSection(const std::string& section) const;

		// Make sure that every function that returns an IConfigSection*,
		// this pointer is added to mPendingInterfaces for later cleanup.
		IConfigSection*		parent();
		IConfigSection*		section(const std::string& section);
		IConfigSection*		firstSection();
		IConfigSection*		nextSection();
		IConfigSection*		nextSimilarSection();
		bool				has(const std::string& property) const;

		//bool				isNull() const;
		IConfigProperty*	get(const std::string& property);
		IConfigProperty*	firstProperty();

	public:
							IXMLConfigSection(TiXmlElement* pElement);
		virtual				~IXMLConfigSection();
};

class IXMLConfigProperty: public IConfigProperty
{
	friend class CConfigSection;
	private:
		TiXmlElement*		mPElement;
		bool				isProperty(TiXmlElement* pElement) const;

	public:
							IXMLConfigProperty(TiXmlElement* pElement);
		virtual				~IXMLConfigProperty();
	protected:
		std::string			name() const;
		std::string 		toString() const;
		void				set(const std::string& value);
		IConfigProperty*	nextProperty();
		bool				isVerbose() const;
};


/**
 * The configuration class reads the values from the specified configuration file.
 *
 *
 */

typedef std::vector<TiXmlElement*>	TiXmlElementList;

class CXMLConfiguration: public CConfiguration
{
	protected:
		/** The XML document */
		TiXmlDocument			mXMLDocument;
		IXMLConfigSection		*mPRootConfigSection;

		/** The filename */
		std::string				mFilename;
		// findXmlNode() searches for the XML nodes that fall into the nodePath (can contain wildcards, slashes, etc).
		// Currently, only two types of paths are supported:
		// 1) subnode/subnode2		(returns subnode2)
		// 2) subnode/subnode2/		(returns all nodes *inside* subnode2)
		bool					findXmlNode(TiXmlElement* rootElement, const std::string& nodePath, TiXmlElementList* resultList);

		void					processIncludes(TiXmlElement* rootNode, TiXmlElement* node, const std::string& filePath);	// Processes <include> nodes
	public:
								CXMLConfiguration();
		virtual					~CXMLConfiguration();

		virtual bool			loadFile(const std::string& filename);
		virtual bool			saveFile(const std::string& filename = "");
		virtual bool			reload();
		virtual std::string		errorStr();
		virtual void			clear();
		virtual CConfigSection	root();

		// DEBUGGING FUNCTIONS
		void print();

	private:
		/* Deny copying this class. Users should copy CConfigSections
		 * instead. */
		CXMLConfiguration(CXMLConfiguration &other) : CConfiguration(other) { }
		CXMLConfiguration &operator=(const CXMLConfiguration &other) { return *this; }
};

#endif /*XMLCONFIGURATION_H_*/
