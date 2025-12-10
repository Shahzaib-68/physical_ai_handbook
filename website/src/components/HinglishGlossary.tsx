import React, { ReactNode } from 'react';

// A simple mapping of English terms to Hinglish equivalents
// In a real implementation, this would be more comprehensive
const HINGLISH_TRANSLATIONS: Record<string, string> = {
  'robot': 'robot',
  'artificial intelligence': 'krtificial intelligence',
  'machine learning': 'machine learning',
  'algorithm': 'algorithm',
  'code': 'code',
  'programming': 'programming',
  'function': 'function',
  'variable': 'variable',
  'loop': 'loop',
  'condition': 'condition',
  'physics': 'bhautiki',
  'simulation': 'simulation',
  'motion': 'gati',
  'velocity': 'veg',
  'acceleration': 'tatvaran',
  'force': 'bal',
  'gravity': 'gurutvakarshan',
  'sensor': 'sensor',
  'actuator': 'actuator',
  'controller': 'niyantrak',
  'kinematics': 'gati vidhan',
  'dynamics': 'gati shashtra'
};

interface HinglishGlossaryProps {
  children: ReactNode;
  enabled?: boolean;
}

const HinglishGlossary: React.FC<HinglishGlossaryProps> = ({ 
  children, 
  enabled = true 
}) => {
  // If Hinglish is disabled, just return the children as is
  if (!enabled) {
    return <>{children}</>;
  }

  // In a real implementation, we would process the text to replace
  // technical terms with Hinglish equivalents
  const processText = (text: string): string => {
    if (typeof text !== 'string') return text;
    
    let processedText = text;
    
    // Replace terms with Hinglish equivalents
    Object.entries(HINGLISH_TRANSLATIONS).forEach(([english, hinglish]) => {
      // Replace with a special marker that can be styled differently
      const regex = new RegExp(`\\b${english}\\b`, 'gi');
      processedText = processedText.replace(regex, `__HINGLISH:${hinglish}__`);
    });
    
    return processedText;
  };

  // Process React nodes to extract and translate text
  const processNode = (node: ReactNode): ReactNode => {
    if (typeof node === 'string') {
      const processed = processText(node);
      
      // Split at the Hinglish markers
      const parts = processed.split(/(__HINGLISH:[^_]+__)/);
      
      return parts.map((part, i) => {
        if (part.startsWith('__HINGLISH:') && part.endsWith('__')) {
          const hinglishTerm = part.slice(11, -2); // Remove the '__HINGLISH:' and '__' parts
          return (
            <span key={i} className="hinglish-term" title={`Aka ${part.slice(11, -2)}`}>
              {hinglishTerm}
            </span>
          );
        }
        return part;
      });
    }
    
    if (React.isValidElement(node)) {
      return React.cloneElement(node, {
        children: React.Children.map(node.props.children, processNode)
      });
    }
    
    if (Array.isArray(node)) {
      return node.map((child, i) => processNode(child));
    }
    
    return node;
  };

  return <>{processNode(children)}</>;
};

export default HinglishGlossary;